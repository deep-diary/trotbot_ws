#ifndef _DEEP_MOTOR_H__
#define _DEEP_MOTOR_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "protocol_motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led/circular_strip.h"
#include "deep_motor_led_state.h"

/**
 * @brief 深度电机管理类 - 用于管理多个电机的状态和ID
 * 
 * 功能：
 * 1. 自动注册和跟踪活跃的电机ID
 * 2. 解析和保存电机状态数据
 * 3. 管理当前活跃电机ID
 * 4. 提供电机状态查询接口
 */

// 最大电机数量（机器狗 4 腿 × 3 关节 = 12）
#define MAX_MOTOR_COUNT 13

// 电机ID未注册标识
#define MOTOR_ID_UNREGISTERED -1

// 电机反馈帧ID掩码 (0x02******)
#define MOTOR_FEEDBACK_MASK 0x02000000
#define MOTOR_FEEDBACK_MASK_SHIFT 24

// 录制功能相关常量
#define MAX_TEACHING_POINTS 300        // 最大录制点数
#define TEACHING_SAMPLE_RATE_MS 50     // 录制采样间隔(ms)
#define INIT_STATUS_RATE_MS 200       // 初始化状态查询间隔(ms)

// 宏定义用于解析29位ID
#define RX_29ID_DISASSEMBLE_MOTOR_ID(id)        (uint8_t)(((id)>>8)&0xFF)
#define RX_29ID_DISASSEMBLE_MASTER_ID(id)       (uint8_t)((id)&0xFF)

class DeepMotor {
private:
    // 已注册的电机ID列表
    int8_t registered_motor_ids_[MAX_MOTOR_COUNT];
    
    // 电机状态数组，索引对应registered_motor_ids_中的位置
    motor_status_t motor_statuses_[MAX_MOTOR_COUNT];
    
    // 电机目标位置（软件侧期望/用于 LED 等），索引与 registered_motor_ids_ 对齐
    float motor_target_angles_[MAX_MOTOR_COUNT];

    // 仅在该电机初始化成功后才允许统计 max_abs_torque / collision，避免把初始化前脏反馈记入峰值
    bool torque_observe_enabled_[MAX_MOTOR_COUNT];

    /**
     * 最近一次成功下发到驱动器的指令缓存（用于去重：与协议 PARAM_* 一致时再发则跳过）。
     * position_known/speed_known/iq_known 为 false 表示尚未通过本类成功下发过对应量。
     */
    struct MotorCommandCache {
        float position_rad;       // PARAM_LOC_REF
        float speed_limit_rad_s;  // PARAM_LIMIT_SPD
        float iq_ref;             // PARAM_IQ_REF（电流/力矩环目标，协议命名）
        bool position_known;
        bool speed_known;
        bool iq_known;
    };
    MotorCommandCache motor_cmd_cache_[MAX_MOTOR_COUNT];
    
    // 当前活跃电机ID
    int8_t active_motor_id_;
    
    // 已注册电机数量
    uint8_t registered_count_;
    
    // 录制功能相关变量
    bool teaching_mode_;                    // 录制模式标志
    bool teaching_data_ready_;              // 录制数据就绪标志
    float teaching_positions_[MAX_TEACHING_POINTS];  // 录制位置数据数组
    uint16_t teaching_point_count_;         // 当前录制点数
    uint16_t current_execute_index_;        // 当前执行索引
    
    // 任务句柄
    TaskHandle_t init_status_task_handle_;  // 初始化状态查询任务句柄
    TaskHandle_t teaching_task_handle_;     // 录制任务句柄
    TaskHandle_t execute_task_handle_;      // 播放任务句柄
    
    // 查找电机ID在注册列表中的索引
    int8_t findMotorIndex(uint8_t motor_id) const;

    
    // 注册新电机ID
    bool registerMotorId(uint8_t motor_id);
    
    // 静态任务函数
    static void initStatusTask(void* parameter);
    static void recordingTask(void* parameter);
    static void playTask(void* parameter);
    
    // 回调函数类型定义
    typedef void (*MotorDataCallback)(uint8_t motor_id, float position, void* user_data);
    
    // 回调函数相关
    MotorDataCallback data_callback_;
    void* callback_user_data_;
    
    // LED状态管理器
    DeepMotorLedState* led_state_manager_;

public:
    /**
     * @brief 构造函数
     * @param led_strip LED灯带控制器指针，如果为nullptr则不启用LED功能
     */
    DeepMotor(CircularStrip* led_strip = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~DeepMotor();
    
    /**
     * @brief 处理CAN帧，自动注册电机并更新状态
     * @param can_frame 接收到的CAN帧
     * @return true 如果是电机反馈帧并处理成功, false 否则
     */
    bool processCanFrame(const CanFrame& can_frame);
    
    /**
     * @brief 获取当前活跃电机ID
     * @return 活跃电机ID，-1表示无活跃电机
     */
    int8_t getActiveMotorId() const;
    
    /**
     * @brief 设置当前活跃电机ID
     * @param motor_id 电机ID
     * @return true 成功, false 失败（电机未注册）
     */
    bool setActiveMotorId(uint8_t motor_id);
    
    /**
     * @brief 获取电机状态（含反馈：实际位置/速度/力矩等）
     * @param motor_id 电机ID
     * @param status 状态结构体指针
     * @return true 成功, false 失败（电机未注册）
     */
    bool getMotorStatus(uint8_t motor_id, motor_status_t* status) const;

    /** 反馈：当前角度 (rad) */
    bool getMotorActualPosition(uint8_t motor_id, float* rad) const;
    /** 反馈：当前角速度 (rad/s) */
    bool getMotorActualSpeed(uint8_t motor_id, float* rad_s) const;
    /** 反馈：当前力矩 (N·m) */
    bool getMotorActualTorque(uint8_t motor_id, float* torque_nm) const;

    /**
     * 最近一次成功下发的设定（与去重缓存一致）；若从未下发过对应项，*known 为 false。
     */
    bool getMotorLastSentPosition(uint8_t motor_id, float* rad, bool* known) const;
    bool getMotorLastSentSpeedLimit(uint8_t motor_id, float* rad_s, bool* known) const;
    bool getMotorLastSentIqRef(uint8_t motor_id, float* iq_ref, bool* known) const;
    
    /**
     * @brief 设置电机目标位置
     * @param motor_id 电机ID
     * @param target_angle 目标角度（弧度）
     * @return true 成功, false 失败（电机未注册）
     */
    bool setMotorTargetAngle(uint8_t motor_id, float target_angle);
    
    /**
     * @brief 获取电机目标位置
     * @param motor_id 电机ID
     * @param target_angle 目标角度输出指针
     * @return true 成功, false 失败（电机未注册）
     */
    bool getMotorTargetAngle(uint8_t motor_id, float* target_angle) const;
    
    /**
     * @brief 设置电机位置（包装方法，同时更新目标位置）
     * @param motor_id 电机ID
     * @param position 目标位置（弧度）
     * @param max_speed 最大速度（弧度/秒）
     * @return true 成功, false 失败
     */
    bool setMotorPosition(uint8_t motor_id, float position, float max_speed = 50.0f);

    /** 仅下发速度限制（PARAM_LIMIT_SPD），用于批量动作前统一限速 */
    bool setMotorSpeedLimit(uint8_t motor_id, float max_speed_rad_s);

    /** 仅下发位置参考（PARAM_LOC_REF），不重复写速度；须保证限速已设或与 init 一致 */
    bool setMotorPositionRefOnly(uint8_t motor_id, float position);

    /**
     * 仅下发电流/力矩环目标（PARAM_IQ_REF，与协议 setCurrent 一致）。
     * 若与上次成功下发值一致则跳过 CAN。
     */
    bool setMotorIqRef(uint8_t motor_id, float iq_ref);

    /**
     * 运控帧下发（位置+速度+Kp+Kd+前馈力矩；力矩编码在 29 位 CAN ID bit8–23，见 protocol_motor）。
     * 会 invalidate 位置/速度参数缓存，避免与后续位置模式指令混淆。
     */
    bool setMotorMitCommand(uint8_t motor_id, float position_rad, float velocity_rad_s, float kp, float kd,
                            float torque_ff = 0.0f);

    /**
     * 若通过 MotorProtocol 直接 reset/改模式等，与本类缓存不一致时调用，强制后续指令重新下发。
     */
    void invalidateMotorCommandCache(uint8_t motor_id);
    
    /**
     * @brief 获取所有已注册电机ID
     * @param motor_ids 输出数组，用于存储电机ID
     * @param max_count 数组最大长度
     * @return 实际返回的电机数量
     */
    uint8_t getRegisteredMotorIds(int8_t* motor_ids, uint8_t max_count) const;
    
    /**
     * @brief 注册电机ID（用于在未收到反馈前即可发送控制指令）
     * @param motor_id 电机ID
     * @return true 成功, false 失败（已满或已存在）
     */
    bool registerMotor(uint8_t motor_id);
    
    /**
     * @brief 检查电机是否已注册
     * @param motor_id 电机ID
     * @return true 已注册, false 未注册
     */
    bool isMotorRegistered(uint8_t motor_id) const;
    
    /**
     * @brief 获取已注册电机数量
     * @return 已注册电机数量
     */
    uint8_t getRegisteredCount() const;
    
    /**
     * @brief 清除所有电机注册信息
     */
    void clearAllMotors();
    
    /**
     * @brief 打印所有电机状态信息（调试用）
     */
    void printAllMotorStatus() const;
    
    /**
     * @brief 开始录制模式
     * @param motor_id 电机ID
     * @return true 成功, false 失败
     * 
     * 功能：
     * 1. 停止电机（便于人工拖动）
     * 2. 启动50ms定时任务发送位置查询
     * 3. 设置录制标志位
     * 4. 清空录制数据数组
     */
    bool startTeaching(uint8_t motor_id);
    
    /**
     * @brief 结束录制模式
     * @return true 成功, false 失败
     * 
     * 功能：
     * 1. 终止录制任务
     * 2. 清除录制标志位
     * 3. 保存录制数据
     */
    bool stopTeaching();
    
    /**
     * @brief 播放录制
     * @param motor_id 电机ID
     * @return true 成功, false 失败
     * 
     * 功能：
     * 1. 使能电机
     * 2. 每隔50ms发送位置数据
     * 3. 按顺序发送录制数据
     * 4. 发送完毕后停止
     */
    bool executeTeaching(uint8_t motor_id);
    
    /**
     * @brief 获取录制状态
     * @return true 录制模式中, false 非录制模式
     */
    bool isTeachingMode() const;
    
    /**
     * @brief 获取录制数据是否就绪
     * @return true 录制数据就绪, false 录制数据未就绪
     */
    bool isTeachingDataReady() const;
    
    /**
     * @brief 获取录制点数
     * @return 录制点数
     */
    uint16_t getTeachingPointCount() const;
    
    /**
     * @brief 启动初始化状态查询任务
     * @param motor_id 电机ID
     * @return true 成功, false 失败
     * 
     * 功能：
     * 1. 启动1秒定时任务发送位置模式查询
     * 2. 便于获取电机最新状态信息
     */
    bool startInitStatusTask(uint8_t motor_id);
    
    /**
     * @brief 停止初始化状态查询任务
     * @return true 成功, false 失败
     */
    bool stopInitStatusTask();
    
    /**
     * @brief 设置电机数据回调函数
     * @param callback 回调函数指针
     * @param user_data 用户数据指针
     */
    void setMotorDataCallback(MotorDataCallback callback, void* user_data = nullptr);
    
    // ========== LED控制接口 ==========
    
    /**
     * @brief 启用电机角度LED指示器
     * @param motor_id 电机ID
     * @param enabled 是否启用
     * @return true 成功, false 失败
     */
    bool enableAngleIndicator(uint8_t motor_id, bool enabled = true);
    
    /**
     * @brief 禁用电机角度LED指示器
     * @param motor_id 电机ID
     * @return true 成功, false 失败
     */
    bool disableAngleIndicator(uint8_t motor_id);
    
    /**
     * @brief 设置电机角度范围
     * @param motor_id 电机ID
     * @param min_angle 最小角度（弧度）
     * @param max_angle 最大角度（弧度）
     * @return true 成功, false 失败
     */
    bool setAngleRange(uint8_t motor_id, float min_angle, float max_angle);
    
    /**
     * @brief 获取电机角度状态
     * @param motor_id 电机ID
     * @return 电机角度状态
     */
    DeepMotorLedState::MotorAngleState getAngleStatus(uint8_t motor_id) const;
    
    /**
     * @brief 获取角度范围
     * @param motor_id 电机ID
     * @return 角度范围
     */
    DeepMotorLedState::AngleRange getAngleRange(uint8_t motor_id) const;
    
    /**
     * @brief 检查角度指示器是否启用
     * @param motor_id 电机ID
     * @return true 启用, false 未启用
     */
    bool isAngleIndicatorEnabled(uint8_t motor_id) const;
    
    /**
     * @brief 停止所有角度指示器
     */
    void stopAllAngleIndicators();
    
    /**
     * @brief 获取LED状态管理器指针（用于高级控制）
     * @return LED状态管理器指针，如果未初始化则返回nullptr
     */
    DeepMotorLedState* getLedStateManager() const;

    // ========== 呼吸灯控制接口 ==========
    
    /**
     * @brief 启用电机呼吸灯效果（正弦信号时使用）
     * @param motor_id 电机ID
     * @param red 红色分量 (0-255)
     * @param green 绿色分量 (0-255)
     * @param blue 蓝色分量 (0-255)
     * @return true 成功, false 失败
     */
    bool enableBreatheEffect(uint8_t motor_id, uint8_t red, uint8_t green, uint8_t blue);
    
    /**
     * @brief 禁用电机呼吸灯效果
     * @param motor_id 电机ID
     * @return true 成功, false 失败
     */
    bool disableBreatheEffect(uint8_t motor_id);
    
    /**
     * @brief 检查电机是否在呼吸灯模式
     * @param motor_id 电机ID
     * @return true 在呼吸灯模式, false 不在
     */
    bool isBreatheEffectEnabled(uint8_t motor_id) const;

    // ========== 软件版本查询接口 ==========
    
    /**
     * @brief 获取电机软件版本号
     * @param motor_id 电机ID
     * @param version 版本号字符串缓冲区
     * @param buffer_size 缓冲区大小
     * @return true 成功, false 失败（电机未注册）
     */
    bool getMotorSoftwareVersion(uint8_t motor_id, char* version, size_t buffer_size) const;

    /**
     * @brief 初始化单个电机（带“置零后反馈校验”与失败立即失能保护）
     * - reset×2 → set_zero×3 → 设置运行模式 → enable
     * - 之后主动触发若干次状态查询以拉取反馈，必须收到反馈且实际角在零位附近，否则立即 reset（停扭）并返回失败
     */
    bool initializeMotor(uint8_t motor_id, float target_velocity_rad_s = 0.0f);
};

#endif // _DEEP_MOTOR_H__
