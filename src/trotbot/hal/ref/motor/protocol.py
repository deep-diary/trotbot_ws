# src/services/hardware_communication/device_protocols/deep_motor_protocol/protocol.py
# DeepMotor 设备协议的核心实现

from __future__ import division, print_function, absolute_import

import struct
import logging
import sys
from typing import Dict, Any, List, Union, Optional
from deepwin.config.config_manager import ConfigManager
from deepwin.data_management.log_manager import LogManager

class DeepMotorProtocol:
    """
    DeepMotor 通信协议实现。
    提供 DeepMotor 无刷电机通信的协议处理功能，包括命令编码和响应解码。
    """
    
    # 协议配置
    _PROTOCOL_CONFIG = {
        "communication": {
            "start_byte": 0xAA,
            "end_byte": 0x55,
            "escape_byte": 0xCC,
            "max_packet_size": 128,
            "timeout": 1.0,
            "use_uart2can": True
        },
        "commands": {
            "motor_control": {"code": 0x01, "parameters": [{"name": "motor_id", "type": "uint8", "description": "电机ID"}, {"name": "angle", "type": "float", "description": "目标角度"}, {"name": "speed", "type": "uint16", "description": "转动速度"}]},
            "get_motor_status": {"code": 0x02, "parameters": [{"name": "motor_id", "type": "uint8", "description": "电机ID"}]},
            "system_reset": {"code": 0x03, "parameters": []},
            "read_sensor": {"code": 0x04, "parameters": [{"name": "sensor_id", "type": "uint8", "description": "传感器ID"}]}
        },
        "responses": {
            "ack": {"code": 0x80, "parameters": [{"name": "command_code", "type": "uint8", "description": "对应的命令代码"}, {"name": "status", "type": "uint8", "description": "状态码（0表示成功）"}]},
            "motor_status": {"code": 0x81, "parameters": [{"name": "motor_id", "type": "uint8", "description": "电机ID"}, {"name": "current_angle", "type": "float", "description": "当前角度"}, {"name": "current_speed", "type": "uint16", "description": "当前速度"}, {"name": "is_moving", "type": "bool", "description": "是否在运动"}]},
            "sensor_data": {"code": 0x82, "parameters": [{"name": "sensor_id", "type": "uint8", "description": "传感器ID"}, {"name": "sensor_value", "type": "float", "description": "传感器值"}]}
        },
        "error_codes": {
            0x00: "成功", 0x01: "命令未知", 0x02: "参数错误", 0x03: "执行失败", 0x04: "设备忙", 0x05: "超时", 0x06: "校验和错误", 0xFF: "未知错误"
        },
        "protocol": {
            "header": "AT",
            "end_bytes": "\r\n",
            "master_id": "0x00fd"
        },
        "motor": {
            "range": {
                "torque": [-10.0, 10.0], "position": [-12.5, 12.5], "velocity": [-65.0, 65.0],
                "kp": [0.0, 500.0], "kd": [0.0, 5.0],
                "joint1": [-3.2, 1.8], "joint2": [-2.4, 0.8], "joint3": [-0.5, 2.0],
                "joint4": [-1.4, 2.4], "joint5": [-1.3, 1.3], "joint6": [-3.14, 3.14]
            },
            "modes": {
                "mit": 0, "position": 1, "velocity": 2, "torque": 3, "zero": 4, "jog": 7
            }
        },
        "index": {
            "RUN_MODE": "0x7005", "IQ_REF": "0x7006", "SPD_REF": "0x700A", "IMIT_TORQUE": "0x700B",
            "CUR_KP": "0x7010", "CUR_KI": "0x7011", "CUR_FILT_GAIN": "0x7014", "LOC_REF": "0x7016",
            "LIMIT_SPD": "0x7017", "LIMIT_CUR": "0x7018", "ARM_LOC_X": "0x8000", "ARM_LOC_Y": "0x8001",
            "CAMERA_ERROR_X": "0x8010", "CAMERA_ERROR_Y": "0x8011", "CAMERA_H_ANGLE": "0x8012",
            "CAMERA_V_ANGLE": "0x8013", "CAMERA_TARGET_DETECTED": "0x8014"
        },
    }

    def __init__(self, log_manager: LogManager):
        """
        初始化协议对象。
        :param log_manager: 全局日志管理器实例。
        """
        self.logger = log_manager.get_logger(__name__)
        
        # 直接使用内联配置
        self.config = self._PROTOCOL_CONFIG 
        
        # 基本协议配置
        protocol_config = self.config.get('protocol', {})
        self.AT_HEADER = protocol_config.get('header', 'AT').encode('ascii')
        self.END_BYTES = protocol_config.get('end_bytes', '\r\n').encode('ascii')
        self.master_id = int(protocol_config.get('master_id', '0x00fd'), 16)

        # 获取参数范围常量
        motor_range = self.config.get('motor', {}).get('range', {})
        self.T_MIN, self.T_MAX = motor_range.get('torque', [-10.0, 10.0])
        self.P_MIN, self.P_MAX = motor_range.get('position', [-12.5, 12.5])
        self.V_MIN, self.V_MAX = motor_range.get('velocity', [-65.0, 65.0])
        self.KP_MIN, self.KP_MAX = motor_range.get('kp', [0.0, 500.0])
        self.KD_MIN, self.KD_MAX = motor_range.get('kd', [0.0, 5.0])
        # Data conversion parameters
        self.POSITION_RANGE = (-4 * 3.14159, 4 * 3.14159)  # -4π ~ 4π
        self.VELOCITY_RANGE = (-30, 30)                     # -30rad/s ~ 30rad/s
        self.TORQUE_RANGE = (-12, 12)                      # -12Nm ~ 12Nm

        # 电机参数索引
        self.index = {}
        for key, value in self.config.get('index', {}).items():
            self.index[key] = int(value, 16)
        
        # 运行模式
        self.modes = self.config.get('motor', {}).get('modes', {})
        
        self.logger.info("DeepMotor Protocol 初始化完成。")

    def _create_frame(self, mode: int, motor_id: int, res: int, data: int, payload: Optional[bytes] = None) -> bytes:
        """
        创建通信帧
        
        Args:
            mode: 命令模式
            motor_id: 电机ID
            res: 保留字段
            data: 数据字段
            payload: 负载数据
            
        Returns:
            bytes: 完整的通信帧
        """
        can_id = (res << 29) | (mode << 24) | (data << 8) | motor_id
        if self.config.get('communication', {}).get('use_uart2can', False):
            can_id = (can_id << 3) + 0x04  # 如果需要使用 USB 转 CAN 模块，需要进行转换
        
        frame = bytearray()
        frame.extend(self.AT_HEADER)
        frame.extend(struct.pack('>I', can_id))
        
        if payload:
            frame.append(len(payload))
            frame.extend(payload)
        else:
            frame.append(0)
        
        frame.extend(self.END_BYTES)
        return bytes(frame)

    def _float_to_uint(self, x: float, x_min: float, x_max: float, bits: int) -> int:
        """
        将浮点数转换为无符号整数
        """
        span = x_max - x_min
        offset = x_min
        x = min(max(x, x_min), x_max)
        return int((x - offset) * ((1 << bits) - 1) / span)

    def _uint_to_float(self, uint: int, x_min: float, x_max: float, bits: int) -> float:
        """
        将无符号整数转换为浮点数
        """
        span = x_max - x_min
        offset = x_min
        return uint * span / ((1 << bits) - 1) + offset

    def _limit_position(self, motor_id: int, position: float) -> float:
        """
        限制电机位置在预设范围内
        """
        joint_name = 'joint' + str(motor_id)
        joint_range = self.config.get('motor', {}).get('range', {}).get(joint_name, [-0.5, 0.5])
        position = min(max(position, joint_range[0]), joint_range[1])
        return position

    def encode_command(self, command_type: str, **kwargs) -> Union[bytes, List[bytes]]:
        """
        将命令类型和参数编码为底层协议命令。
        :param command_type: 命令类型。
        :param kwargs: 命令参数字典。
        :return: 编码后的命令字节或命令字节列表。
        """
        self.logger.debug(f"DeepMotorProtocol: 编码命令 '{command_type}' 参数: {kwargs}")
        
        try:
            if command_type == 'enable_motor': #  41 54 18 07 e8 34 08 00 00 00 00 00 00 00 00 0d 0a
                motor_id = kwargs.get('motor_id', 1)
                return self._create_frame(3, motor_id, 0, self.master_id) 
            elif command_type == 'disable_motor':  # 41 54 20 07 e8 34 08 00 00 00 00 00 00 00 00 0d 0a
                motor_id = kwargs.get('motor_id', 1)
                return self._create_frame(4, motor_id, 0, self.master_id)
            elif command_type == 'reset_motor':
                motor_id = kwargs.get('motor_id', 1)
                return self._create_frame(4, motor_id, 0, self.master_id)
            elif command_type == 'zero_motor':
                motor_id = kwargs.get('motor_id', 1)
                payload = bytearray([0] * 8)
                payload[0] = 1
                payload[1] = 1
                return self._create_frame(6, motor_id, 0, self.master_id, payload)
            elif command_type == 'set_motor_mode': # 位置模式：41 54 90 07 e8 2c 08 05 70 00 00 01 00 00 00 0d 0a
                motor_id = kwargs.get('motor_id', 1)
                run_mode = kwargs.get('run_mode', 1)
                return self.create_motor_mode_frame(motor_id,run_mode)
            elif command_type == 'set_motor_mit_mode':
                motor_id = kwargs.get('motor_id', 1)
                torque = kwargs.get('torque', 0.0)
                position = kwargs.get('position', 0.0)
                speed = kwargs.get('speed', 0.0)
                kp = kwargs.get('kp', 0.0)
                kd = kwargs.get('kd', 0.0)
                data_val = self._float_to_uint(torque, self.T_MIN, self.T_MAX, 16)
                payload = bytearray(8)
                pos_uint = self._float_to_uint(position, self.P_MIN, self.P_MAX, 16)
                spd_uint = self._float_to_uint(speed, self.V_MIN, self.V_MAX, 16)
                kp_uint = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16)
                kd_uint = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16)
                payload[0:2] = struct.pack('<H', pos_uint)
                payload[2:4] = struct.pack('<H', spd_uint)
                payload[4:6] = struct.pack('<H', kp_uint)
                payload[6:8] = struct.pack('<H', kd_uint)
                return self._create_frame(1, motor_id, 0, data_val, payload)
            elif command_type == 'write_motor_param':
                motor_id = kwargs.get('motor_id', 1)
                index = kwargs.get('index')
                value = kwargs.get('value')
                if index is None or value is None:
                    raise ValueError("write_motor_param 命令必须提供 'index' 和 'value' 参数")
                if isinstance(index, str):
                    index = self.index.get(index)
                    if index is None:
                        raise ValueError("未知的索引名称: %s" % index)
                payload = bytearray(8)
                payload[0:2] = struct.pack('<H', index)
                payload[4:8] = struct.pack('<f', float(value))
                return self._create_frame(0x12, motor_id, 0, self.master_id, payload)
            elif command_type == 'read_motor_param':
                motor_id = kwargs.get('motor_id', 1)
                index = kwargs.get('index')
                if index is None:
                    raise ValueError("read_motor_param 命令必须提供 'index' 参数")
                if isinstance(index, str):
                    index = self.index.get(index)
                    if index is None:
                        raise ValueError("未知的索引名称: %s" % index)
                payload = bytearray(8)
                payload[0:2] = struct.pack('<H', index)
                return self._create_frame(0x11, motor_id, 0, self.master_id, payload)
            elif command_type == 'jog_motor':
                motor_id = kwargs.get('motor_id', 1)
                speed = kwargs.get('speed', 0)
                index_run_mode = self.index['RUN_MODE']
                run_mode = self.modes.get('jog', 7)
                payload = bytearray(8)
                payload[0:2] = struct.pack('<H', index_run_mode)
                payload[4] = run_mode
                payload[5] = 0x01  # 1: 启用 jog
                speed = min(max(speed, -30), 30)
                scaled_speed = int((speed + 30) / 60 * 65535)
                payload[6:8] = struct.pack('>H', scaled_speed)
                return self._create_frame(0x12, motor_id, 0, self.master_id, payload)
            elif command_type == 'stop_jog_motor':
                motor_id = kwargs.get('motor_id', 1)
                index_run_mode = self.index['RUN_MODE']
                run_mode = self.modes.get('jog', 7)
                payload = bytearray(8)
                payload[0:2] = struct.pack('<H', index_run_mode)
                payload[4] = run_mode
                payload[5] = 0x00  # 0: 禁用 jog
                payload[6:8] = struct.pack('>H', 0x7fff) # 将速度设置为中间值以有效停止
                return self._create_frame(0x12, motor_id, 0, self.master_id, payload)
            elif command_type == 'init_motor':
                motor_id = kwargs.get('motor_id', 1)
                # frames = [
                #     self.AT_HEADER + b'+AT' + self.END_BYTES,
                #     self._create_frame(4, motor_id, 0, self.master_id), # Reset
                #     self._create_frame(6, motor_id, 0, self.master_id, bytearray([1, 1, 0, 0, 0, 0, 0, 0])), # Zero
                #     self._create_frame(3, motor_id, 0, self.master_id), # Enable
                #     self.encode_command('set_motor_mode', motor_id=motor_id, value=self.modes.get('position', 1)) # Set Position Mode
                # ]
                frames = self.create_motor_init_frame(motor_id)
                return frames
            elif command_type == 'init_all_motors':
                motor_ids = kwargs.get('motor_ids', [])
                frames = []
                for motor_id in motor_ids:
                    frames.extend(self.encode_command('init_motor', motor_id=motor_id))
                return frames
            elif command_type == 'reset_all_motors':
                motor_ids = kwargs.get('motor_ids', [])
                return [self._create_frame(4, motor_id, 0, self.master_id) for motor_id in motor_ids]
            elif command_type == 'set_motor_position':
                motor_id = kwargs.get('motor_id', 1)
                position = kwargs.get('position')
                if position is None: raise ValueError("set_motor_position 命令需要 'position' 参数。")
                loc_index = self.index['LOC_REF']
                # limited_position = self._limit_position(motor_id, position) # 对单电机来说，取消限制
                limited_position = position
                return self.encode_command('write_motor_param', motor_id=motor_id, index=loc_index, value=limited_position)
            elif command_type == 'set_all_motors_position':
                motor_ids = kwargs.get('motor_ids', [])
                positions = kwargs.get('positions', [])
                if len(motor_ids) != len(positions):
                    raise ValueError("电机ID和位置列表长度必须匹配。")
                frames = []
                for i, motor_id in enumerate(motor_ids):
                    frames.append(self.encode_command('set_motor_position', motor_id=motor_id, position=positions[i]))
                return frames
            elif command_type == 'set_motor_pos_speed':
                motor_id = kwargs.get('motor_id', 1)
                position = kwargs.get('position')
                speed = kwargs.get('speed')
                if position is None or speed is None: raise ValueError("set_motor_pos_speed 命令需要 'position' 和 'speed' 参数。")
                loc_index = self.index['LOC_REF']
                spd_index = self.index['LIMIT_SPD']
                limited_position = self._limit_position(motor_id, position)
                position_frame = self.encode_command('write_motor_param', motor_id=motor_id, index=loc_index, value=limited_position)
                speed_frame = self.encode_command('write_motor_param', motor_id=motor_id, index=spd_index, value=speed)
                return [position_frame, speed_frame]
            elif command_type == 'set_all_motors_pos_speed':
                motor_ids = kwargs.get('motor_ids', [])
                positions = kwargs.get('positions', [])
                speeds = kwargs.get('speeds', [])
                if not (len(motor_ids) == len(positions) == len(speeds)):
                    raise ValueError("电机ID、位置和速度列表长度必须匹配。")
                frames = []
                for i, motor_id in enumerate(motor_ids):
                    frames.extend(self.encode_command('set_motor_pos_speed', motor_id=motor_id, position=positions[i], speed=speeds[i]))
                return frames
            else:
                raise ValueError(f"不支持的命令类型: {command_type}")
        except Exception as e:
            raise ValueError(f"编码命令 '{command_type}' 失败: {e}")

    def decode_response(self, data: bytes) -> Dict[str, Any]:
        """
        将接收到的字节序列解码为响应数据
        
        Args:
            data: 接收到的字节序列
            
        Returns:
            Dict[str, Any]: 解码后的响应数据
        """
        try:
            can_id, payload = self._ser2can(data)
            ext_can_id_info = self._decode_ext_can_id(can_id)
            response_data = self._decode_can_data(payload)
            response_data.update(ext_can_id_info)
            
            # 根据故障信息设置 error_code
            error_code = 0
            if ext_can_id_info.get('flt_uninitialized', 0):
                error_code |= 0x01
            if ext_can_id_info.get('flt_hall_encoding', 0):
                error_code |= 0x02
            if ext_can_id_info.get('flt_magnetic_encoding', 0):
                error_code |= 0x04
            if ext_can_id_info.get('flt_over_temperature', 0):
                error_code |= 0x08
            if ext_can_id_info.get('flt_over_current', 0):
                error_code |= 0x10
            if ext_can_id_info.get('flt_voltage_drop', 0):
                error_code |= 0x20
            
            response_data['error_code'] = error_code
            response_data['success'] = True
            
            # 对于其他响应模式，只返回基本信息
            return response_data
            
        except Exception as e:
            self.logger.error(f"解析响应失败: {str(e)}, 原始数据: {data.hex()}")
            return {'success': False, 'error': f"解析响应失败: {str(e)}"}
        
    def _ser2can(self, frame_bytes: bytes):
        """
        将串口数据解析为 CAN 帧组件。
        """
        # 解析CAN ID（4字节），先向右移3位, 具体根据协议决定
        arbitration_id = int.from_bytes(frame_bytes[0:4], byteorder='big')

        # 如果使用 USB 转 CAN 模块，需要进行转换
        if self.config.get('communication', {}).get('use_uart2can', True):
            arbitration_id = arbitration_id >> 3
        
        # 解析数据长度（1字节）
        data_length = frame_bytes[4]
        
        # 检查数据长度是否合理
        if data_length > 8:  # CAN 2.0 标准帧最大数据长度为8字节
            self.logger.warning(f"DeepMotorProtocol: 数据长度超出范围: {data_length}")
            return

        # 检查接收到的数据长度是否足够
        expected_length = 5 + data_length  # 5 = 4(CANID) + 1(Len)
        if len(frame_bytes) < expected_length:
            self.logger.warning(f"DeepMotorProtocol: 数据不完整，期望 {expected_length} 字节，实际 {len(frame_bytes)} 字节")
            return

        # 提取数据部分
        data_bytes = frame_bytes[5:5+data_length]


        # # 假设所有 CAN ID 都是标准 ID (非扩展 ID)，实际项目中需要根据 CANID 范围判断
        # is_extended_id = True

        self.logger.info(f"DeepMotorProtocol: 解析到 CAN 帧: ID=0x{arbitration_id:X}, Len={data_length}, Data={data_bytes.hex()}")
        # 发射解析后的 CAN 帧组件，CanBusCommunicator 将会接收并进一步处理
        return arbitration_id, data_bytes

    def _decode_ext_can_id(self, ext_can_id):
        """
        解析 CAN ID
        """
        response_mode = (ext_can_id >> 24) & 0xFF
        motor_can_id = (ext_can_id >> 8) & 0xFF  # Bit8~Bit15: 当前电机CANID
        fault_info = (ext_can_id >> 16) & 0x3F  # Bit21~Bit16: 故障信息
        mode_state = (ext_can_id >> 22) & 0x3  # Bit22~Bit23: 模式状态

        # Parse fault information
        faults = {
            "flt_uninitialized": (fault_info >> 5) & 0x1,
            "flt_hall_encoding": (fault_info >> 4) & 0x1,
            "flt_magnetic_encoding": (fault_info >> 3) & 0x1,
            "flt_over_temperature": (fault_info >> 2) & 0x1,
            "flt_over_current": (fault_info >> 1) & 0x1,
            "flt_voltage_drop": fault_info & 0x1
        }

        # 解析模式状态
        mode_states = {
            0: "ResetMode",
            1: "CaliMode",
            2: "RunMode"
        }
        ext_can_id_info = {
            "response_mode": response_mode,
            "motor_can_id": motor_can_id,
            "mode_state": mode_states[mode_state]
        }
        ext_can_id_info.update(faults)  # 更新故障信息

        # 记录
        self.logger.info(f"DeepMotorProtocol: 解析到 CAN ID: {ext_can_id_info}")


        return ext_can_id_info

    def _decode_can_data(self, data_bytes):
        """
        Update motor position
        
        Args:
            data_bytes: 8-byte feedback data
            
        Raises:
            ValueError: Incorrect data length
        """
        if len(data_bytes) != 8:
            raise ValueError("Feedback data must be 8 bytes")
            
        # Parse position data (Byte 0-1)
        position_raw = struct.unpack('>H', data_bytes[0:2])[0]
        position_raw = position_raw - 32767
        position = self._scale_value(position_raw, -32768, 32767,
                                                self.POSITION_RANGE[0],
                                                self.POSITION_RANGE[1])
        
        # Parse velocity data (Byte 2-3)
        velocity_raw = struct.unpack('>H', data_bytes[2:4])[0]
        velocity_raw = velocity_raw - 32767
        velocity = self._scale_value(velocity_raw, -32768, 32767,
                                               self.VELOCITY_RANGE[0],
                                               self.VELOCITY_RANGE[1])
        
        # Parse torque data (Byte 4-5)
        torque_raw = struct.unpack('>H', data_bytes[4:6])[0]
        torque_raw = torque_raw - 32767
        torque = self._scale_value(torque_raw, -32768, 32767,
                                             self.TORQUE_RANGE[0],
                                             self.TORQUE_RANGE[1])
        
        # Parse temperature data (Byte 6-7)
        temperature = struct.unpack('>H', data_bytes[6:8])[0]
        temperature = temperature / 10

        
        self.logger.debug("Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f" % 
                         (position, velocity, torque, temperature))
        
        response_data = {
            'position': position,
            'velocity': velocity,
            'torque': torque,
            'temperature': temperature
        }
        return response_data
    
    def _scale_value(self, value, in_min, in_max, out_min, out_max):
        """
        Update motor temperature
        
        Args:
            value: Input value
            in_min: Input minimum value
            in_max: Input maximum value
            out_min: Output minimum value
            out_max: Output maximum value
            
        Returns:
            float: Mapped value
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def create_AT_frame(self):
        # Send 'AT+AT' command
        frame = [0x41, 0x54, 0x2B, 0x41, 0x54, 0x0D, 0x0A]
        return frame

    def create_frame(self, mode, motor_id, res, data, payload=None):
        """
        Create a communication frame
        
        Args:
            mode: Command mode
            motor_id: Motor ID
            res: Reserved field
            data: Data field
            payload: Payload data
            
        Returns:
            bytearray: Complete communication frame
        """
        can_id = (res << 29) | (mode << 24) | (data << 8) | motor_id
        if self.config.get('communication', {}).get('use_uart2can', True):
            can_id = (can_id << 3) + 0x04  # If you need to use the usb to can module, you need to convert
        
        frame = bytearray()
        frame.extend(self.AT_HEADER)
        frame.extend(struct.pack('>I', can_id))
        
        if payload:
            frame.append(len(payload))
            frame.extend(payload)
        else:
            frame.append(0)
        
        frame.extend(self.END_BYTES)
        return frame

    def create_motor_enable_frame(self, motor_id):
        """
        Create a motor enable frame (mode 3)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        return self.create_frame(3, motor_id, 0, self.master_id)

    def create_motor_reset_frame(self, motor_id):
        """
        Create a motor reset frame (mode 4)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        return self.create_frame(4, motor_id, 0, self.master_id)

    def create_motor_zero_frame(self, motor_id):
        """
        Create a motor zero frame (mode 6)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        payload = bytearray([0] * 8)
        payload[0] = 1
        payload[1] = 1
        return self.create_frame(6, motor_id, 0, self.master_id, payload)

    def create_motor_mode_frame(self, motor_id, run_mode):
        """
        Create a motor mode setting frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            index: Parameter index
            run_mode: Operating mode
            
        Returns:
            bytearray: Communication frame
        """
        index = self.index['RUN_MODE']
        payload = bytearray(8)
        payload[0:2] = [index & 0xFF, (index >> 8) & 0xFF]
        payload[4] = run_mode
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_jog_frame(self, motor_id, jog_speed):
        """
        Create a motor jog mode setting frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            jog_speed: Jog speed
            
        Returns:
            bytearray: Communication frame
        """
        index = self.index['RUN_MODE']    # write
        run_mode = self.modes.get('jog', 7)  # jog mode
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        payload[4] = run_mode
        payload[5] = 0x01  # 1: enable jog, 0: disable jog
        jog_speed = min(max(jog_speed, -30), 30)
        scaled_speed = int((jog_speed + 30) / 60 * 65535)
        payload[6:8] = struct.pack('>H', scaled_speed)
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_jog_stop_frame(self, motor_id):
        """
        Create a motor jog mode stop frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        index = self.index['RUN_MODE']    # write
        run_mode = self.modes.get('jog', 7)  # jog mode
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        payload[4] = run_mode
        payload[5] = 0x00  # 1: enable jog, 0: disable jog
        payload[6:8] = struct.pack('>H', 0x7fff)
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_write_frame(self, motor_id, index, value):
        """
        Create a motor parameter write frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            index: Parameter index
            value: Parameter value
            
        Returns:
            bytearray: Communication frame
        """
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        payload[4:8] = struct.pack('<f', float(value))
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_read_frame(self, motor_id, index):
        """
        Create a motor parameter read frame (mode 0x11)
        
        Args:
            motor_id: Motor ID
            index: Parameter index
            
        Returns:
            bytearray: Communication frame
        """
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        return self.create_frame(0x11, motor_id, 0, self.master_id, payload)
    
    def create_motor_mit_mode_frame(self, motor_id, torque, position, speed, kp, kd):
        """
        Create a motor MIT mode control frame (mode 1)
        
        Args:
            motor_id: Motor ID
            torque: Torque value
            position: Target position
            speed: Target speed
            kp: Position loop gain
            kd: Velocity loop gain
            
        Returns:
            bytearray: Communication frame
        """
        data = self.float_to_uint(torque, self.T_MIN, self.T_MAX, 16)
        payload = bytearray(8)
        pos_uint = self.float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        spd_uint = self.float_to_uint(speed, self.V_MIN, self.V_MAX, 16)
        kp_uint = self.float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16)
        kd_uint = self.float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16)
        
        payload[0:2] = struct.pack('<H', pos_uint)
        payload[2:4] = struct.pack('<H', spd_uint)
        payload[4:6] = struct.pack('<H', kp_uint)
        payload[6:8] = struct.pack('<H', kd_uint)
        
        return self.create_frame(1, motor_id, 0, data, payload)
    
    def create_motor_init_frame(self, motor_id):
        """
        Create a motor initialization frame (including enable, zero and mode setting)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            list: Communication frame list
        """
        return [
            self.create_AT_frame(),
            self.create_motor_reset_frame(motor_id),
            self.create_motor_zero_frame(motor_id),
            self.create_motor_mode_frame(motor_id, self.modes.get('position', 1)),
            self.create_motor_enable_frame(motor_id)
        ]
    
    def create_motor_init_frame_all(self, motor_ids):
        """
        Create initialization frames for all motors
        
        Args:
            motor_ids: Motor ID list
            
        Returns:
            list: Communication frame list
        """
        frames = []
        for motor_id in motor_ids:
            frames.extend(self.create_motor_init_frame(motor_id))
        return frames
    
    def create_motor_reset_frame_all(self, motor_ids):
        """
        Create reset frames for all motors
        
        Args:
            motor_ids: Motor ID list
            
        Returns:
            list: Communication frame list
        """
        return [self.create_motor_reset_frame(motor_id) for motor_id in motor_ids]
    
    def create_motor_pos_frame(self, motor_id, position):
        # Calculate the position index and speed index
        loc_index = self.index['LOC_REF']
        # Create and send the position control frame
        return self.create_motor_write_frame(motor_id, loc_index, self.limit_position(motor_id, position))
    
    def create_motor_pos_frame_all(self, motor_ids, positions):
        frames = []
        # Check if the list lengths match
        if len(motor_ids) != len(positions):
            self.logger.error("Motor ID and position list lengths do not match")
            return frames
        
        # Create a position control frame for each motor
        for i, motor_id in enumerate(motor_ids):
            position = positions[i]
            frames.append(self.create_motor_pos_frame(motor_id, self.limit_position(motor_id, position)))

        return frames

    def create_motor_pos_spd_frame(self, motor_id, position, speed):
        """
        Create a motor position and speed control frame
        
        Args:
            motor_id: Motor ID
            position: Target position
            speed: Target speed
            
        Returns:
            bytearray: Communication frame
        """
        # Calculate the position index and speed index
        loc_index = self.index['LOC_REF']
        spd_index = self.index['LIMIT_SPD']
        
        # Create and send the position control frame
        position_frame = self.create_motor_write_frame(motor_id, loc_index, self.limit_position(motor_id, position))
        
        # Create and send the speed control frame
        speed_frame = self.create_motor_write_frame(motor_id, spd_index, speed)
        
        # Return two frames, the caller needs to send them in order
        return [position_frame, speed_frame]
    
    def create_motor_spd_frame(self, motor_id, speed):
        # Calculate the speed index
        spd_index = self.index['LIMIT_SPD']
        # Create and send the speed control frame
        return self.create_motor_write_frame(motor_id, spd_index, speed)

    def create_motor_torque_frame(self, motor_id, torque):
        # Calculate the torque index
        torque_index = self.index['LIMIT_CUR']
        # Create and send the torque control frame
        return self.create_motor_write_frame(motor_id, torque_index, torque)

    def create_motor_frame_all_pos_spd(self, motor_ids, positions, speeds):
        """
        Create multiple motor position and speed control frames
        
        Args:
            motor_ids: Motor ID list
            positions: Target position list
            speeds: Target speed list
            
        Returns:
            list: Communication frame list
        """
        frames = []
        
        # Check if the list lengths match
        if len(motor_ids) != len(positions) or len(motor_ids) != len(speeds):
            self.logger.error("Motor ID, position and speed list lengths do not match")
            return frames
        
        # Create a control frame for each motor
        for i, motor_id in enumerate(motor_ids):
            position = positions[i]
            speed = speeds[i]
            # Get the position and speed control frame
            pos_spd_frames = self.create_motor_pos_spd_frame(motor_id, position, speed)
            frames.extend(pos_spd_frames)
        
        return frames

    def limit_position(self, motor_id, position):
        # Get the corresponding joint name according to the motor_id
        joint_name = 'joint' + str(motor_id)
        # Get the joint range
        joint_range = self.config.get('motor', {}).get('range', {}).get(joint_name, [-0.5, 0.5])
        # Limit the position within the joint range
        position = min(max(position, joint_range[0]), joint_range[1])
        return position
    
    def create_motor_sinwave_test_frame(self, motor_id, amplitude, frequency, start_stop):
        # 41 54 90 07 e8 34 08 03 70 00 00 00 00 80 3f 0d 0a # Set amplitude
        # 41 54 90 07 e8 34 08 02 70 00 00 00 00 80 3f 0d 0a # Set frequency
        # 41 54 90 07 e8 34 08 01 70 00 00 01 00 00 00 0d 0a # Start sine test
        # 41 54 90 07 e8 34 08 01 70 00 00 00 00 00 00 0d 0a # Stop sine test
        # 41 54 20 07 e8 34 08 00 00 00 00 00 00 00 00 0d 0a # Reset motor

        # 41 54 98 07 e8 34 08 1b 82 30 02 a0 42 32 0e 0d 0a # Load parameter table
        pass

    def create_motor_scope_disp_frame(self, motor_id, frequency, channel, start_stop):
    # 41 54 50 07 e8 0c 08 14 00 11 00 00 10 0e 00 0d 0a    Set sampling frequency
    # 41 54 50 0f e8 0c 08 16 30 16 30 16 30 16 30 0d 0a    Set channel
    # 41 54 50 17 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a    Start sampling
    # 41 54 50 1f e8 0c 08 00 00 00 00 00 00 00 00 0d 0a    Stop sampling
        pass

    def float_to_uint(self, x, x_min, x_max, bits):
        """
        Convert a float to an unsigned integer

        Args:
            x: Input float
            x_min: Minimum value
            x_max: Maximum value
            bits: Number of bits
            
        Returns:
            int: Converted unsigned integer
        """
        span = x_max - x_min
        offset = x_min
        x = min(max(x, x_min), x_max)
        return int((x - offset) * ((1 << bits) - 1) / span)

    def uint_to_float(self, uint, x_min, x_max, bits):
        """
        Convert an unsigned integer to a float
        
        Args:
            uint: Unsigned integer
            x_min: Minimum value
            x_max: Maximum value
            bits: Number of bits
            
        Returns:
            float: Converted float
        """
        span = x_max - x_min
        offset = x_min
        return uint * span / ((1 << bits) - 1) + offset
