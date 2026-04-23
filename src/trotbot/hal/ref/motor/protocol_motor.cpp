#include "protocol_motor.h"
#include "motor_config.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MotorProtocol";

// 将软件版本号转为字符串格式（假设前4字节为ASCII码，后4字节为日期，最后1字节为版本号）
static inline void RX_DATA_DISASSEMBLE_VERSION_STR(const uint8_t data[8], char* out_str, size_t out_len) {
    // 例子：0x32 0x30 0x32 0x35 0x31 0x30 0x30 0x30 0x07
    // 前4字节为ASCII码，后4字节为日期，最后1字节为版本号
    // 例如："20251007"
    if (out_len < 9) { // 8位数字+1终止符
        if (out_len > 0) out_str[0] = '\0';
        return;
    }
    // 假设data[0]~data[3]为年份，data[4]~data[7]为日期
    // 但实际例子是8字节数字字符串
    for (int i = 0; i < 8; ++i) {
        out_str[i] = (char)data[i];
    }
    out_str[8] = '\0';
}


bool MotorProtocol::enableMotor(uint8_t motor_id) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.identifier = buildCanId(motor_id, MOTOR_CMD_ENABLE);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;
    
    // 数据全为0
    for (int i = 0; i < 8; i++) {
        frame.data[i] = 0;
    }
    
    return sendCanFrame(frame);
}

bool MotorProtocol::resetMotor(uint8_t motor_id) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.identifier = buildCanId(motor_id, MOTOR_CMD_RESET);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;
    
    // 数据全为0
    for (int i = 0; i < 8; i++) {
        frame.data[i] = 0;
    }
    frame.data[1] = 0xC0;  // 如果第1个字节是c0， 则会返回软件版本号
    
    return sendCanFrame(frame);
}

bool MotorProtocol::setMotorZero(uint8_t motor_id) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.identifier = buildCanId(motor_id, MOTOR_CMD_SET_ZERO);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;
    
    // 只需将第0位设置为1
    frame.data[0] = 1;
    
    return sendCanFrame(frame);
}

bool MotorProtocol::controlMotor(uint8_t motor_id, float position, float velocity, float kp, float kd,
                                 float torque_ff) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));

    // EL05：通信类型 1，bit8–23 为前馈力矩（±6 N·m），bit0–7 为电机 ID；勿再用固定主机字节 0xFD 占位
    frame.identifier = buildMitControlCanId(motor_id, torque_ff);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;

    // 8 字节：目标角、角速度、Kp、Kd；各 16 位 **大端**（高字节在前）。
    uint16_t pos_data = floatToUint16(position, P_MIN, P_MAX, 16);
    frame.data[0] = (pos_data >> 8) & 0xFF;
    frame.data[1] = pos_data & 0xFF;

    uint16_t vel_data = floatToUint16(velocity, V_MIN, V_MAX, 16);
    frame.data[2] = (vel_data >> 8) & 0xFF;
    frame.data[3] = vel_data & 0xFF;

    uint16_t kp_data = floatToUint16(kp, KP_MIN, KP_MAX, 16);
    frame.data[4] = (kp_data >> 8) & 0xFF;
    frame.data[5] = kp_data & 0xFF;

    uint16_t kd_data = floatToUint16(kd, KD_MIN, KD_MAX, 16);
    frame.data[6] = (kd_data >> 8) & 0xFF;
    frame.data[7] = kd_data & 0xFF;

    return sendCanFrame(frame);
}

uint32_t MotorProtocol::buildMitControlCanId(uint8_t motor_id, float torque_ff_nm) {
    uint16_t torque_u16 = floatToUint16(torque_ff_nm, T_FF_MIN, T_FF_MAX, 16);
    uint32_t id = 0;
    id |= ((uint32_t)MOTOR_CMD_CONTROL << 24);
    id |= ((uint32_t)torque_u16 << 8);
    id |= (uint32_t)motor_id;
    return id & 0x1FFFFFFFu;
}

bool MotorProtocol::initializeMotor(uint8_t motor_id, float target_velocity_rad_s) {
    ESP_LOGI(TAG, "开始初始化电机%d（target_velocity=%.2f rad/s）", motor_id, target_velocity_rad_s);

    resetMotor(motor_id);
    ESP_LOGI(TAG, "步骤0: 设置电机%d零位（发送两帧增强可靠性）", motor_id);
    bool zero_ok = false;
    for (int k = 0; k < 2; ++k) {
        if (setMotorZero(motor_id)) {
            zero_ok = true;
        } else {
            ESP_LOGW(TAG, "设置电机%d零位第%d帧发送失败", motor_id, k + 1);
        }
        if (k == 0) {
            vTaskDelay(pdMS_TO_TICKS(8));
        }
    }
    if (!zero_ok) {
        ESP_LOGE(TAG, "设置电机%d零位失败（两帧均失败）", motor_id);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

#if DEEP_DOG_USE_MIT_WALK
    ESP_LOGI(TAG, "步骤1: 设置电机%d为运控模式", motor_id);
    if (!setMotorControlMode(motor_id)) {
        ESP_LOGE(TAG, "设置电机%d为运控模式失败", motor_id);
        return false;
    }
#else
    ESP_LOGI(TAG, "步骤1: 设置电机%d为位置模式", motor_id);
    if (!setMotorPositionMode(motor_id)) {
        ESP_LOGE(TAG, "设置电机%d为位置模式失败", motor_id);
        return false;
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "步骤2: 使能电机%d", motor_id);
    if (!enableMotor(motor_id)) {
        ESP_LOGE(TAG, "使能电机%d失败", motor_id);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

#if DEEP_DOG_USE_MIT_WALK
    // 发送初始 MIT 保持帧
    if (!controlMotor(motor_id, 0.0f, 0.0f, DEEP_DOG_MIT_DEFAULT_KP, DEEP_DOG_MIT_DEFAULT_KD,
                      DEEP_DOG_MIT_DEFAULT_TAU_FF)) {
        ESP_LOGE(TAG, "MIT init: 电机%d 初始运控保持帧发送失败", motor_id);
        return false;
    }
    ESP_LOGI(TAG, "电机%d初始化完成（MIT，未发送初始保持帧）", motor_id);
#else
    ESP_LOGI(TAG, "电机%d初始化完成（位置模式，未配置速度上限，使用驱动默认）", motor_id);
#endif
    ESP_LOGI(TAG, "电机%d初始化完成，状态查询任务将由DeepMotor类管理", motor_id);
    return true;
}

bool MotorProtocol::setPosition(uint8_t motor_id, float position, float max_speed) {
    // 先设置速度限制
    if (!setMotorParameter(motor_id, PARAM_LIMIT_SPD, max_speed)) {
        return false;
    }
    
    // 再设置位置指令
    return setMotorParameter(motor_id, PARAM_LOC_REF, position);
}

bool MotorProtocol::setPositionOnly(uint8_t motor_id, float position) { // 只设置位置，不设置速度限制
    // 设置位置指令
    return setMotorParameter(motor_id, PARAM_LOC_REF, position);
}

bool MotorProtocol::setSpeed(uint8_t motor_id, float speed) {
    return setMotorParameter(motor_id, PARAM_LIMIT_SPD, speed);
}

bool MotorProtocol::setCurrent(uint8_t motor_id, float current) {
    return setMotorParameter(motor_id, PARAM_IQ_REF, current);
}

bool MotorProtocol::changeMotorMode(uint8_t motor_id, motor_run_mode_t mode) {
    return setMotorParameter(motor_id, PARAM_RUN_MODE, (float)mode);
}

bool MotorProtocol::setMotorRunMode(uint8_t motor_id, uint8_t mode) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.identifier = buildCanId(motor_id, MOTOR_CMD_SET_PARAM);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;
    
    // 设置参数索引为运行模式 (0x7005)
    uint16_t parameter = PARAM_RUN_MODE;
    frame.data[0] = parameter & 0xFF;
    frame.data[1] = (parameter >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    
    // 设置模式值
    frame.data[4] = mode;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    ESP_LOGD(TAG, "设置电机%d运行模式为: %d", motor_id, mode);
    return sendCanFrame(frame);
}

bool MotorProtocol::setMotorControlMode(uint8_t motor_id) {
    return setMotorRunMode(motor_id, MOTOR_CTRL_MODE);
}

bool MotorProtocol::setMotorCurrentMode(uint8_t motor_id) {
    return setMotorRunMode(motor_id, MOTOR_CURRENT_MODE);
}

bool MotorProtocol::setMotorSpeedMode(uint8_t motor_id) {
    return setMotorRunMode(motor_id, MOTOR_SPEED_MODE);
}

bool MotorProtocol::setMotorPositionMode(uint8_t motor_id) {
    return setMotorRunMode(motor_id, MOTOR_POS_MODE);
}

bool MotorProtocol::sendRunModeForStatusQuery(uint8_t motor_id) {
#if DEEP_DOG_USE_MIT_WALK
    return setMotorControlMode(motor_id);
#else
    return setMotorPositionMode(motor_id);
#endif
}

bool MotorProtocol::setMotorParameter(uint8_t motor_id, motor_param_t param, float value) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.identifier = buildCanId(motor_id, MOTOR_CMD_SET_PARAM);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;
    
    // 设置参数索引（大端序）
    frame.data[0] = param & 0xFF;
    frame.data[1] = (param >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    
    // 设置参数值（大端序）
    union { float f; uint32_t i; } value_union;
    value_union.f = value;
    uint32_t value_int = value_union.i;
    frame.data[4] = value_int & 0xFF;
    frame.data[5] = (value_int >> 8) & 0xFF;
    frame.data[6] = (value_int >> 16) & 0xFF;
    frame.data[7] = (value_int >> 24) & 0xFF;
    
    return sendCanFrame(frame);
}

bool MotorProtocol::setMotorParameterRaw(uint8_t motor_id, motor_param_t param, const uint8_t data_bytes[4]) {
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.identifier = buildCanId(motor_id, MOTOR_CMD_SET_PARAM);
    frame.extd = 1;
    frame.rtr = 0;
    frame.ss = 0;
    frame.self = 0;
    frame.dlc_non_comp = 0;
    frame.data_length_code = 8;
    
    // 设置参数索引（大端序）
    frame.data[0] = param & 0xFF;
    frame.data[1] = (param >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    
    // 直接设置参数值（使用传入的4字节数据）
    frame.data[4] = data_bytes[0];
    frame.data[5] = data_bytes[1];
    frame.data[6] = data_bytes[2];
    frame.data[7] = data_bytes[3];
    
    return sendCanFrame(frame);
}

// 开始sin 正弦信号
bool MotorProtocol::startSinSignal(uint8_t motor_id, float amp, float freq) {

    //0. 先关闭电机使能
    resetMotor(motor_id);

    //1. 设置幅度
    setMotorParameter(motor_id, PARAM_SIN_AMP, amp);

    //2. 设置频率
    setMotorParameter(motor_id, PARAM_SIN_FREQ, freq);

    //3. 设置使能开启
    uint8_t data_bytes[4] = {1,0,0,0};

    setMotorParameterRaw(motor_id, PARAM_SIN_ENABLE, data_bytes);

    //3. 使能电机
    enableMotor(motor_id);

    return true;
}

// 停止sin 正弦信号
bool MotorProtocol::stopSinSignal(uint8_t motor_id) {

    //1. 先关闭电机使能
    resetMotor(motor_id);

    //2. 设置使能关闭
    uint8_t data_bytes[4] = {0,0,0,0};
    setMotorParameterRaw(motor_id, PARAM_SIN_ENABLE, data_bytes);

    return true;
}


uint32_t MotorProtocol::buildCanId(uint8_t motor_id, motor_cmd_t cmd) {
    uint32_t id = 0;
    id |= (cmd << 24);           // 5位模式
    id |= (MOTOR_MASTER_ID << 8); // 8位主机ID
    id |= motor_id;              // 8位电机ID
    return id;
}

bool MotorProtocol::sendCanFrame(const CanFrame& frame) {
#if DEEP_DOG_CAN_TX_HEX_LOG
    ESP_LOGI(TAG, "CAN TX ext id=0x%08lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)frame.identifier, (unsigned)frame.data_length_code, frame.data[0], frame.data[1],
             frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#else
    ESP_LOGD(TAG, "CAN TX id=0x%08lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)frame.identifier, (unsigned)frame.data_length_code, frame.data[0], frame.data[1],
             frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#endif
    bool ok = ESP32Can.writeFrame(frame, MOTOR_CAN_TIMEOUT_MS);
    if (!ok) {
        ESP_LOGE(TAG, "CAN 发送失败 (无ACK或超时 %dms)", (int)MOTOR_CAN_TIMEOUT_MS);
    }
    return ok;
}

uint16_t MotorProtocol::floatToUint16(float value, float min_val, float max_val, int bits) {
    // 限制值在范围内
    if (value > max_val) value = max_val;
    else if (value < min_val) value = min_val;
    
    float span = max_val - min_val;
    float offset = min_val;
    return (uint16_t)((value - offset) * ((float)((1 << bits) - 1)) / span);
}

void MotorProtocol::parseMotorData(const CanFrame& can_frame, motor_status_t* status) {
    const uint8_t cmd_type = RX_29ID_DISASSEMBLE_CMD_TYPE(can_frame.identifier);
    status->master_id = RX_29ID_DISASSEMBLE_MASTER_ID(can_frame.identifier);
    status->motor_id = RX_29ID_DISASSEMBLE_MOTOR_ID(can_frame.identifier);
    ESP_LOGD(TAG, "电机%d命令类型: %d", status->motor_id, cmd_type);

    status->error_status = RX_29ID_DISASSEMBLE_ERR_STA(can_frame.identifier);
    status->hall_error = RX_29ID_DISASSEMBLE_HALL_ERR(can_frame.identifier);
    status->magnet_error = RX_29ID_DISASSEMBLE_MAGNET_ERR(can_frame.identifier);
    status->temp_error = RX_29ID_DISASSEMBLE_TEMP_ERR(can_frame.identifier);
    status->current_error = RX_29ID_DISASSEMBLE_CURRENT_ERR(can_frame.identifier);
    status->voltage_error = RX_29ID_DISASSEMBLE_VOLTAGE_ERR(can_frame.identifier);
    status->mode_status = (motor_mode_t)RX_29ID_DISASSEMBLE_MODE_STA(can_frame.identifier);
    
    // 解析数据
    switch (cmd_type) {
        case MOTOR_CMD_FEEDBACK: {
            // 与运控指令相同：8 字节内各量 16 位大端、线性映射（EL05）
            const uint16_t raw_a = RX_DATA_DISASSEMBLE_CUR_ANGLE(can_frame.data);
            const uint16_t raw_s = RX_DATA_DISASSEMBLE_CUR_SPEED(can_frame.data);
            const uint16_t raw_t = RX_DATA_DISASSEMBLE_CUR_TORQUE(can_frame.data);
            const float span_p = (P_MAX - P_MIN);
            const float span_v = (V_MAX - V_MIN);
            const float span_t = (T_FF_MAX - T_FF_MIN);
            status->current_angle = P_MIN + (float)raw_a * span_p / 65535.0f;
            status->current_speed = V_MIN + (float)raw_s * span_v / 65535.0f;
            status->current_torque = T_FF_MIN + (float)raw_t * span_t / 65535.0f;
            status->current_temp = RX_DATA_DISASSEMBLE_CUR_TEMP(can_frame.data) / 10.0f;
            break;
        }
        case MOTOR_CMD_VERSION:
            RX_DATA_DISASSEMBLE_VERSION_STR(can_frame.data, status->version, sizeof(status->version));
            // 打印 can_frame.data
            // 以16进制显示版本号原始数据
            ESP_LOGI(TAG, "电机%d软件版本原始数据: %02X %02X %02X %02X %02X %02X %02X %02X", 
                     status->motor_id,
                     can_frame.data[0], can_frame.data[1], can_frame.data[2], can_frame.data[3],
                     can_frame.data[4], can_frame.data[5], can_frame.data[6], can_frame.data[7]);
            ESP_LOGI(TAG, "电机%d软件版本: %s", status->motor_id, status->version);
            break;
        default:
            break;
    }
}
