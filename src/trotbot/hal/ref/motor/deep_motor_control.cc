#include "deep_motor_control.h"
#include "protocol_motor.h"
#include <esp_log.h>

#define TAG "DeepMotorControl"

static void RegisterMotorMcpToolsImpl(McpServer& mcp_server, DeepMotor* deep_motor) {
    // CAN控制工具
    // mcp_server.AddTool("self.can.send_motor_position", "发送电机位置控制指令", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
    //     Property("position", kPropertyTypeInteger, 200, -314, 314)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
    //     int position_int = properties["position"].value<int>();
    //     float position = position_int / 100.0f; // 转换为浮点数（保留2位小数）
        
    //     if (!deep_motor) {
    //         ESP_LOGW(TAG, "深度电机管理器未初始化");
    //         return std::string("深度电机管理器未初始化");
    //     }
        
    //     // 使用电机类方法发送位置控制指令，自动保存目标位置
    //     if (deep_motor->setMotorPosition(motor_id, position, 10.0f)) {
    //         ESP_LOGI(TAG, "发送电机位置控制指令成功 - 电机ID: %d, 位置: %.2f 弧度", motor_id, position);
    //         return std::string("电机ID " + std::to_string(motor_id) + " 位置设置成功: " + std::to_string(position) + " 弧度");
    //     } else {
    //         ESP_LOGE(TAG, "发送电机位置控制指令失败 - 电机ID: %d", motor_id);
    //         return std::string("电机ID " + std::to_string(motor_id) + " 位置设置失败");
    //     }
    // });

    // // 电机速度控制工具
    // mcp_server.AddTool("self.can.send_motor_speed", "发送电机速度控制指令", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
    //     Property("speed", kPropertyTypeInteger, 0, -300, 300)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
    //     int speed_int = properties["speed"].value<int>();
    //     float speed = speed_int / 10.0f; // 转换为浮点数（保留1位小数）
        
    //     if (MotorProtocol::setSpeed(motor_id, speed)) {
    //         ESP_LOGI(TAG, "发送电机速度控制指令成功 - 电机ID: %d, 速度: %.1f rad/s", motor_id, speed);
    //         return true;
    //     } else {
    //         ESP_LOGE(TAG, "发送电机速度控制指令失败 - 电机ID: %d", motor_id);
    //         return false;
    //     }
    // });

    // 电机使能工具
    mcp_server.AddTool("self.can.enable_motor", "激活电机", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        int motor_id = properties["motor_id"].value<int>();
        
        if (MotorProtocol::enableMotor(motor_id)) {
            ESP_LOGI(TAG, "电机使能成功 - 电机ID: %d", motor_id);
            return true;
        } else {
            ESP_LOGE(TAG, "电机使能失败 - 电机ID: %d", motor_id);
            return false;
        }
    });

    // 电机停止工具
    mcp_server.AddTool("self.can.reset_motor", "停止电机", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        int motor_id = properties["motor_id"].value<int>();
        
        if (MotorProtocol::resetMotor(motor_id)) {
            ESP_LOGI(TAG, "电机停止成功 - 电机ID: %d", motor_id);
            return true;
        } else {
            ESP_LOGE(TAG, "电机停止失败 - 电机ID: %d", motor_id);
            return false;
        }
    });

    // 位置/角速/kp/kd/τ：一律「参数÷10 = 物理量」，与语音整数对齐
    mcp_server.AddTool(
        "self.can.control_motor",
        "MIT 运控（先 registerMotor）。position_x10 等参数=物理量×10，设备上再÷10；不是用户念的数字原样放大。"
        "例：目标 2rad 必须填 20；1rad→10；勿把「2弧度」填成 200（会超 ±125）。角速 2rad/s→20；kp=0.5→5；kd=0.5→5。"
        "默认：除扭矩外均为物理量 1.0（position/velocity/kp/kd 的 _x10 默认 10，tau_ff 默认 0）。"
        "position_x10 ±125；velocity_x10 ±500（÷10 后 ±50rad/s）；kp_x10 0～5000→kp 0～500；kd_x10 0～50→kd 0～5。",
        PropertyList(std::vector<Property>{
            Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
            Property("position_x10", kPropertyTypeInteger, 10, -125, 125),
            Property("velocity_x10", kPropertyTypeInteger, 10, -500, 500),
            Property("kp_x10", kPropertyTypeInteger, 10, 0, 5000),
            Property("kd_x10", kPropertyTypeInteger, 10, 0, 50),
            Property("tau_ff_x10", kPropertyTypeInteger, 0, -60, 60),
        }),
        [deep_motor](const PropertyList& properties) -> ReturnValue {
            const int motor_id = properties["motor_id"].value<int>();
            const int pos_raw = properties["position_x10"].value<int>();
            const float position = static_cast<float>(pos_raw) / 10.0f;
            const float velocity = properties["velocity_x10"].value<int>() / 10.0f;
            const float kp = properties["kp_x10"].value<int>() / 10.0f;
            const float kd = properties["kd_x10"].value<int>() / 10.0f;
            const float tau_ff = properties["tau_ff_x10"].value<int>() / 10.0f;

            if (!deep_motor) {
                return std::string("DeepMotor 未初始化");
            }
            const uint8_t mid = static_cast<uint8_t>(motor_id);
            if (!deep_motor->isMotorRegistered(mid)) {
                if (!deep_motor->registerMotor(mid)) {
                    return std::string("registerMotor 失败（槽位已满或错误）");
                }
            }
            if (deep_motor->setMotorMitCommand(mid, position, velocity, kp, kd, tau_ff)) {
                ESP_LOGI(TAG, "MIT 下发成功 id=%d pos_x10=%d -> p=%.3f rad v=%.3f kp=%.2f kd=%.2f", motor_id, pos_raw,
                         position, velocity, kp, kd);
                return std::string("MIT 下发成功: id=" + std::to_string(motor_id));
            }
            ESP_LOGE(TAG, "MIT 下发失败 id=%d", motor_id);
            return std::string("MIT 下发失败 id=" + std::to_string(motor_id));
        });

    // 电机状态查询工具
    mcp_server.AddTool("self.motor.get_status", "获取电机状态", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        int motor_id = properties["motor_id"].value<int>();
        
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        if (!deep_motor->isMotorRegistered(motor_id)) {
            ESP_LOGW(TAG, "电机ID %d 未注册", motor_id);
            return std::string("电机ID " + std::to_string(motor_id) + " 未注册");
        }
        
        motor_status_t status;
        if (deep_motor->getMotorStatus(motor_id, &status)) {
            std::string result = "电机ID " + std::to_string(motor_id) + " 状态:\n";
            result += "  角度: " + std::to_string(status.current_angle) + " rad\n";
            result += "  速度: " + std::to_string(status.current_speed) + " rad/s\n";
            result += "  扭矩: " + std::to_string(status.current_torque) + " N·m\n";
            result += "  温度: " + std::to_string(status.current_temp) + "°C";
            
            ESP_LOGI(TAG, "电机ID %d 状态: 角度=%.3f rad, 速度=%.3f rad/s, 扭矩=%.3f N·m, 温度=%.1f°C", 
                     motor_id, status.current_angle, status.current_speed, 
                     status.current_torque, status.current_temp);
            return result;
        }
        return std::string("获取电机ID " + std::to_string(motor_id) + " 状态失败");
    });

    // 获取所有已注册电机ID工具
    // mcp_server.AddTool("self.motor.list", "获取所有已注册电机ID", PropertyList(), [deep_motor](const PropertyList&) -> ReturnValue {
    //     if (!deep_motor) {
    //         ESP_LOGW(TAG, "深度电机管理器未初始化");
    //         return std::string("深度电机管理器未初始化");
    //     }
        
    //     int8_t motor_ids[MAX_MOTOR_COUNT];
    //     uint8_t count = deep_motor->getRegisteredMotorIds(motor_ids, MAX_MOTOR_COUNT);
        
    //     ESP_LOGI(TAG, "已注册电机数量: %d", count);
    //     for (int i = 0; i < count; i++) {
    //         ESP_LOGI(TAG, "电机ID: %d", motor_ids[i]);
    //     }
        
    //     // 构建返回字符串
    //     std::string result = "已注册电机数量: " + std::to_string(count) + "\n";
    //     if (count > 0) {
    //         result += "电机ID列表: ";
    //         for (int i = 0; i < count; i++) {
    //             if (i > 0) result += ", ";
    //             result += std::to_string(motor_ids[i]);
    //         }
    //     } else {
    //         result += "暂无已注册的电机";
    //     }
        
    //     return result;
    // });

    // 设置活跃电机工具
    // mcp_server.AddTool("self.motor.set_active", "设置活跃电机ID", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (!deep_motor) {
    //         ESP_LOGW(TAG, "深度电机管理器未初始化");
    //         return std::string("深度电机管理器未初始化");
    //     }
        
    //     if (deep_motor->setActiveMotorId(motor_id)) {
    //         ESP_LOGI(TAG, "设置活跃电机ID为: %d", motor_id);
    //         return std::string("成功设置活跃电机ID为: " + std::to_string(motor_id));
    //     } else {
    //         ESP_LOGW(TAG, "设置活跃电机ID失败: %d", motor_id);
    //         return std::string("设置活跃电机ID失败: " + std::to_string(motor_id) + " (电机可能未注册)");
    //     }
    // });

    // 打印所有电机状态工具
    mcp_server.AddTool("self.motor.print_all", "打印所有电机状态", PropertyList(), [deep_motor](const PropertyList&) -> ReturnValue {
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        deep_motor->printAllMotorStatus();
        return std::string("已打印所有电机状态到日志");
    });

    // ========== 电机模式控制工具 ==========
    
    // // 设置电机运控模式
    // mcp_server.AddTool("self.motor.set_control_mode", "设置电机为运控模式", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (MotorProtocol::setMotorControlMode(motor_id)) {
    //         ESP_LOGI(TAG, "设置电机%d为运控模式成功", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为运控模式成功");
    //     } else {
    //         ESP_LOGE(TAG, "设置电机%d为运控模式失败", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为运控模式失败");
    //     }
    // });

    // // 设置电机位置模式
    // mcp_server.AddTool("self.motor.set_position_mode", "设置电机为位置模式", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (MotorProtocol::setMotorPositionMode(motor_id)) {
    //         ESP_LOGI(TAG, "设置电机%d为位置模式成功", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为位置模式成功");
    //     } else {
    //         ESP_LOGE(TAG, "设置电机%d为位置模式失败", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为位置模式失败");
    //     }
    // });

    // // 设置电机零位
    // mcp_server.AddTool("self.motor.set_zero_position", "设置电机零位", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (MotorProtocol::setMotorZero(motor_id)) {
    //         ESP_LOGI(TAG, "设置电机%d零位成功", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "零位成功");
    //     } else {
    //         ESP_LOGE(TAG, "设置电机%d零位失败", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "零位失败");
    //     }
    // });

    // // 设置电机速度模式
    // mcp_server.AddTool("self.motor.set_speed_mode", "设置电机为速度模式", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (MotorProtocol::setMotorSpeedMode(motor_id)) {
    //         ESP_LOGI(TAG, "设置电机%d为速度模式成功", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为速度模式成功");
    //     } else {
    //         ESP_LOGE(TAG, "设置电机%d为速度模式失败", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为速度模式失败");
    //     }
    // });

    // // 设置电机电流模式
    // mcp_server.AddTool("self.motor.set_current_mode", "设置电机为电流模式", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (MotorProtocol::setMotorCurrentMode(motor_id)) {
    //         ESP_LOGI(TAG, "设置电机%d为电流模式成功", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为电流模式成功");
    //     } else {
    //         ESP_LOGE(TAG, "设置电机%d为电流模式失败", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为电流模式失败");
    //     }
    // });

    // // 通用模式设置工具
    // mcp_server.AddTool("self.motor.set_mode", "设置电机运行模式", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
    //     Property("mode", kPropertyTypeInteger, 0, 0, 3)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
    //     int mode = properties["mode"].value<int>();
        
    //     const char* mode_names[] = {"运控模式", "位置模式", "速度模式", "电流模式"};
        
    //     if (MotorProtocol::setMotorRunMode(motor_id, mode)) {
    //         ESP_LOGI(TAG, "设置电机%d为%s成功", motor_id, mode_names[mode]);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为" + mode_names[mode] + "成功");
    //     } else {
    //         ESP_LOGE(TAG, "设置电机%d为%s失败", motor_id, mode_names[mode]);
    //         return std::string("设置电机" + std::to_string(motor_id) + "为" + mode_names[mode] + "失败");
    //     }
    // });

    // ========== 电机初始化工具 ==========
    
    // // 电机初始化工具（位置模式）
    // mcp_server.AddTool("self.motor.initialize", "初始化电机（位置模式）", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
    //     Property("max_speed", kPropertyTypeInteger, 300, 10, 300)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
    //     int max_speed_int = properties["max_speed"].value<int>();
    //     float max_speed = max_speed_int / 10.0f; // 转换为浮点数（保留1位小数）
        
    //     if (MotorProtocol::initializeMotor(motor_id, max_speed)) {
    //         ESP_LOGI(TAG, "电机%d初始化成功，最大速度: %.1f rad/s", motor_id, max_speed);
    //         return std::string("电机" + std::to_string(motor_id) + "初始化成功，最大速度: " + std::to_string(max_speed) + " rad/s");
    //     } else {
    //         ESP_LOGE(TAG, "电机%d初始化失败", motor_id);
    //         return std::string("电机" + std::to_string(motor_id) + "初始化失败");
    //     }
    // });

    // 初始化（按编译配置选择 MIT/位置模式）
    mcp_server.AddTool("self.motor.initialize", "初始化电机（按当前编译配置）：reset、写零、切模式、使能。max_speed÷10=rad/s 作为目标速度意图参数。", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
        // 默认 10 → 1.0 rad/s，须在 [10,500] 内（÷10 后 ≤50 rad/s），否则 Property 构造抛异常会导致整机 abort 重启
        Property("max_speed", kPropertyTypeInteger, 10, 10, 500)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        const int motor_id = properties["motor_id"].value<int>();
        const int max_speed_int = properties["max_speed"].value<int>();
        const float target_velocity_rad_s = max_speed_int / 10.0f;
        if (!deep_motor) {
            return std::string("DeepMotor 未初始化");
        }
        const uint8_t mid = static_cast<uint8_t>(motor_id);
        if (!deep_motor->isMotorRegistered(mid)) {
            if (!deep_motor->registerMotor(mid)) {
                return std::string("registerMotor 失败");
            }
        }
        if (deep_motor->initializeMotor(mid, target_velocity_rad_s)) {
            ESP_LOGI(TAG, "电机%d 初始化成功（带反馈校验），target_velocity=%.2f rad/s", motor_id, target_velocity_rad_s);
            return std::string("电机 " + std::to_string(motor_id) + " 初始化成功，target_velocity=" +
                   std::to_string(target_velocity_rad_s) + " rad/s");
        }
        ESP_LOGE(TAG, "电机%d MIT 初始化失败", motor_id);
        return std::string("电机 " + std::to_string(motor_id) + " 初始化失败");
    });

    // ========== 状态查询任务工具 ==========
    
    // 启动状态查询任务
    mcp_server.AddTool("self.motor.start_status_task", "启动电机状态查询任务", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        int motor_id = properties["motor_id"].value<int>();
        
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        if (deep_motor->startInitStatusTask(motor_id)) {
            ESP_LOGI(TAG, "启动电机%d状态查询任务成功", motor_id);
            return std::string("启动电机" + std::to_string(motor_id) + "状态查询任务成功");
        } else {
            ESP_LOGE(TAG, "启动电机%d状态查询任务失败", motor_id);
            return std::string("启动电机" + std::to_string(motor_id) + "状态查询任务失败");
        }
    });

    // 停止状态查询任务
    mcp_server.AddTool("self.motor.stop_status_task", "停止电机状态查询任务", PropertyList(), [deep_motor](const PropertyList&) -> ReturnValue {
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        if (deep_motor->stopInitStatusTask()) {
            ESP_LOGI(TAG, "停止电机状态查询任务成功");
            return std::string("停止电机状态查询任务成功");
        } else {
            ESP_LOGW(TAG, "停止电机状态查询任务失败");
            return std::string("停止电机状态查询任务失败");
        }
    });

    // ========== 录制功能工具 ==========
    
    // 开始录制
    mcp_server.AddTool("self.motor.start_recording", "开始录制模式", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        int motor_id = properties["motor_id"].value<int>();
        
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        if (deep_motor->startTeaching(motor_id)) {
            ESP_LOGI(TAG, "开始录制模式成功，电机ID: %d", motor_id);
            return std::string("开始录制模式成功，电机ID: " + std::to_string(motor_id) + "，可以开始拖动电机");
        } else {
            ESP_LOGE(TAG, "开始录制模式失败，电机ID: %d", motor_id);
            return std::string("开始录制模式失败，电机ID: " + std::to_string(motor_id));
        }
    });

    // 结束录制
    mcp_server.AddTool("self.motor.stop_recording", "结束录制模式", PropertyList(), [deep_motor](const PropertyList&) -> ReturnValue {
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        if (deep_motor->stopTeaching()) {
            uint16_t point_count = deep_motor->getTeachingPointCount();
            ESP_LOGI(TAG, "结束录制模式成功，共记录 %d 个录制点", point_count);
            return std::string("结束录制模式成功，共记录 " + std::to_string(point_count) + " 个录制点");
        } else {
            ESP_LOGE(TAG, "结束录制模式失败");
            return std::string("结束录制模式失败");
        }
    });

    // 播放录制
    mcp_server.AddTool("self.motor.play_recording", "播放录制", PropertyList(std::vector<Property>{
        Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    }), [deep_motor](const PropertyList& properties) -> ReturnValue {
        int motor_id = properties["motor_id"].value<int>();
        
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        if (deep_motor->executeTeaching(motor_id)) {
            uint16_t point_count = deep_motor->getTeachingPointCount();
            ESP_LOGI(TAG, "播放录制成功，电机ID: %d，总点数: %d", motor_id, point_count);
            return std::string("播放录制成功，电机ID: " + std::to_string(motor_id) + "，总点数: " + std::to_string(point_count));
        } else {
            ESP_LOGE(TAG, "播放录制失败，电机ID: %d", motor_id);
            return std::string("播放录制失败，电机ID: " + std::to_string(motor_id));
        }
    });

    // 获取录制状态
    mcp_server.AddTool("self.motor.get_recording_status", "获取录制状态", PropertyList(), [deep_motor](const PropertyList&) -> ReturnValue {
        if (!deep_motor) {
            ESP_LOGW(TAG, "深度电机管理器未初始化");
            return std::string("深度电机管理器未初始化");
        }
        
        bool is_teaching = deep_motor->isTeachingMode();
        bool data_ready = deep_motor->isTeachingDataReady();
        uint16_t point_count = deep_motor->getTeachingPointCount();
        
        std::string result = "录制状态:\n";
        result += "  录制模式: " + std::string(is_teaching ? "进行中" : "未启动") + "\n";
        result += "  数据就绪: " + std::string(data_ready ? "是" : "否") + "\n";
        result += "  录制点数: " + std::to_string(point_count);
        
        ESP_LOGI(TAG, "录制状态 - 模式: %s, 数据就绪: %s, 点数: %d", 
                 is_teaching ? "进行中" : "未启动", 
                 data_ready ? "是" : "否", 
                 point_count);
        
        return result;
    });

    // // ========== 正弦信号控制工具 ==========
    
    // // 开始正弦信号
    // mcp_server.AddTool("self.motor.start_sin_signal", "开始电机正弦信号", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255),
    //     Property("amplitude", kPropertyTypeInteger, 200, 1, 1000),  // 幅度 (0.01-10.00)
    //     Property("frequency", kPropertyTypeInteger, 20, 1, 100)     // 频率 (0.1-10.0 Hz)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
    //     int amplitude_int = properties["amplitude"].value<int>();
    //     int frequency_int = properties["frequency"].value<int>();
        
    //     float amplitude = amplitude_int / 100.0f;  // 转换为浮点数（保留2位小数）
    //     float frequency = frequency_int / 10.0f;   // 转换为浮点数（保留1位小数）
        
    //     if (MotorProtocol::startSinSignal(motor_id, amplitude, frequency)) {
    //         ESP_LOGI(TAG, "开始正弦信号成功 - 电机ID: %d, 幅度: %.2f, 频率: %.1f Hz", motor_id, amplitude, frequency);
            
    //         // 启用呼吸灯效果（绿色呼吸灯表示正弦信号运行中）
    //         // 金色呼吸灯 (RGB: 255, 215, 0)
    //         if (deep_motor && deep_motor->enableBreatheEffect(motor_id, 255, 215, 0)) {
    //             ESP_LOGI(TAG, "电机%d呼吸灯效果已启用", motor_id);
    //         }
            
    //         return std::string("开始正弦信号成功 - 电机ID: " + std::to_string(motor_id) + 
    //                          ", 幅度: " + std::to_string(amplitude) + 
    //                          ", 频率: " + std::to_string(frequency) + " Hz" +
    //                          " (呼吸灯已启用)");
    //     } else {
    //         ESP_LOGE(TAG, "开始正弦信号失败 - 电机ID: %d", motor_id);
    //         return std::string("开始正弦信号失败 - 电机ID: " + std::to_string(motor_id));
    //     }
    // });

    // // 停止正弦信号
    // mcp_server.AddTool("self.motor.stop_sin_signal", "停止电机正弦信号", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (MotorProtocol::stopSinSignal(motor_id)) {
    //         ESP_LOGI(TAG, "停止正弦信号成功 - 电机ID: %d", motor_id);
            
    //         // 禁用呼吸灯效果，切换回角度指示器
    //         if (deep_motor && deep_motor->disableBreatheEffect(motor_id)) {
    //             ESP_LOGI(TAG, "电机%d呼吸灯效果已禁用，切换回角度指示器", motor_id);
    //         }
            
    //         return std::string("停止正弦信号成功 - 电机ID: " + std::to_string(motor_id) + 
    //                          " (呼吸灯已禁用，切换回角度指示器)");
    //     } else {
    //         ESP_LOGE(TAG, "停止正弦信号失败 - 电机ID: %d", motor_id);
    //         return std::string("停止正弦信号失败 - 电机ID: " + std::to_string(motor_id));
    //     }
    // });

    // ========== 电机角度LED指示器工具 ==========
    
    // 启用电机角度指示器
    // mcp_server.AddTool("self.motor.enable_angle_indicator", "启用电机角度LED指示器（默认已启用）", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)  // 电机ID (1-255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (deep_motor && deep_motor->enableAngleIndicator(motor_id, true)) {
    //         ESP_LOGI(TAG, "启用电机%d角度指示器成功", motor_id);
    //         return std::string("启用电机" + std::to_string(motor_id) + "角度指示器成功（注意：电机初始化时已默认启用所有角度指示器）");
    //     } else {
    //         ESP_LOGW(TAG, "启用电机%d角度指示器失败", motor_id);
    //         return std::string("启用电机" + std::to_string(motor_id) + "角度指示器失败");
    //     }
    // });

    // // 禁用电机角度指示器
    // mcp_server.AddTool("self.motor.disable_angle_indicator", "禁用电机角度LED指示器", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)  // 电机ID (1-255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (deep_motor && deep_motor->disableAngleIndicator(motor_id)) {
    //         ESP_LOGI(TAG, "禁用电机%d角度指示器成功", motor_id);
    //         return std::string("禁用电机" + std::to_string(motor_id) + "角度指示器成功");
    //     } else {
    //         ESP_LOGW(TAG, "禁用电机%d角度指示器失败", motor_id);
    //         return std::string("禁用电机" + std::to_string(motor_id) + "角度指示器失败");
    //     }
    // });

    // // 设置电机角度范围
    // mcp_server.AddTool("self.motor.set_angle_range", "设置电机角度范围", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255),  // 电机ID (1-255)
    //     Property("min_angle", kPropertyTypeInteger, -314, -314, 314),  // 最小角度 (0.01弧度)
    //     Property("max_angle", kPropertyTypeInteger, 314, -314, 314)    // 最大角度 (0.01弧度)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
    //     int min_angle_int = properties["min_angle"].value<int>();
    //     int max_angle_int = properties["max_angle"].value<int>();
        
    //     float min_angle = min_angle_int / 100.0f;  // 转换为弧度
    //     float max_angle = max_angle_int / 100.0f;  // 转换为弧度
        
    //     if (deep_motor && deep_motor->setAngleRange(motor_id, min_angle, max_angle)) {
    //         ESP_LOGI(TAG, "设置电机%d角度范围: [%.2f, %.2f] rad", motor_id, min_angle, max_angle);
    //         return std::string("设置电机" + std::to_string(motor_id) + "角度范围: [" + 
    //                          std::to_string(min_angle) + ", " + std::to_string(max_angle) + "] rad");
    //     } else {
    //         ESP_LOGW(TAG, "设置电机%d角度范围失败", motor_id);
    //         return std::string("设置电机" + std::to_string(motor_id) + "角度范围失败");
    //     }
    // });

    // // 获取电机角度状态
    // mcp_server.AddTool("self.motor.get_angle_status", "获取电机角度状态", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)  // 电机ID (1-255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (deep_motor) {
    //         auto angle_state = deep_motor->getAngleStatus(motor_id);
    //         auto angle_range = deep_motor->getAngleRange(motor_id);
    //         bool indicator_enabled = deep_motor->isAngleIndicatorEnabled(motor_id);
            
    //         std::string result = "电机" + std::to_string(motor_id) + "角度状态:\n";
    //         result += "  当前角度: " + std::to_string(angle_state.current_angle) + " rad\n";
    //         result += "  目标角度: " + std::to_string(angle_state.target_angle) + " rad\n";
    //         result += "  是否移动: " + std::string(angle_state.is_moving ? "是" : "否") + "\n";
    //         result += "  是否有错误: " + std::string(angle_state.is_error ? "是" : "否") + "\n";
    //         result += "  角度范围: [" + std::to_string(angle_range.min_angle) + ", " + 
    //                  std::to_string(angle_range.max_angle) + "] rad\n";
    //         result += "  范围有效: " + std::string(angle_range.is_valid ? "是" : "否") + "\n";
    //         result += "  指示器启用: " + std::string(indicator_enabled ? "是" : "否");
            
    //         ESP_LOGI(TAG, "获取电机%d角度状态成功", motor_id);
    //         return result;
    //     } else {
    //         ESP_LOGW(TAG, "深度电机管理器未初始化");
    //         return std::string("深度电机管理器未初始化");
    //     }
    // });


    // ========== 软件版本查询工具 ==========
    
    // 获取电机软件版本号
    // mcp_server.AddTool("self.motor.get_software_version", "获取电机软件版本号", PropertyList(std::vector<Property>{
    //     Property("motor_id", kPropertyTypeInteger, 1, 1, 255)  // 电机ID (1-255)
    // }), [deep_motor](const PropertyList& properties) -> ReturnValue {
    //     int motor_id = properties["motor_id"].value<int>();
        
    //     if (!deep_motor) {
    //         ESP_LOGW(TAG, "深度电机管理器未初始化");
    //         return std::string("深度电机管理器未初始化");
    //     }
        
    //     if (!deep_motor->isMotorRegistered(motor_id)) {
    //         ESP_LOGW(TAG, "电机ID %d 未注册", motor_id);
    //         return std::string("电机ID " + std::to_string(motor_id) + " 未注册");
    //     }
        
    //     char version[16]; // 版本号缓冲区
    //     if (deep_motor->getMotorSoftwareVersion(motor_id, version, sizeof(version))) {
    //         std::string result = "电机ID " + std::to_string(motor_id) + " 软件版本: " + std::string(version);
    //         ESP_LOGI(TAG, "获取电机ID %d 软件版本成功: %s", motor_id, version);
    //         return result;
    //     } else {
    //         ESP_LOGW(TAG, "获取电机ID %d 软件版本失败", motor_id);
    //         return std::string("获取电机ID " + std::to_string(motor_id) + " 软件版本失败");
    //     }
    // });

    ESP_LOGI(TAG, "深度电机控制MCP工具注册完成，包含模式控制、初始化、录制、正弦信号、角度指示器、软件版本查询等功能");
}

void RegisterMotorMcpTools(McpServer& mcp_server, DeepMotor* deep_motor) {
    RegisterMotorMcpToolsImpl(mcp_server, deep_motor);
}

DeepMotorControl::DeepMotorControl(DeepMotor* deep_motor, McpServer& mcp_server)
    : deep_motor_(deep_motor) {
    RegisterMotorMcpTools(mcp_server, deep_motor);
}
