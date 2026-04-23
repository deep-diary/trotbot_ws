#pragma once

#include "led/circular_strip.h"
#include <functional>
#include <vector>

// 前向声明，避免循环依赖
class DeepMotor;

/**
 * @brief 电机角度LED状态管理器
 * 
 * 将电机实际角度映射到24颗LED的环形灯带上
 * 用蓝色灯珠表示当前角度位置
 */
class DeepMotorLedState {
public:
    // LED效果类型
    enum class LedEffect {
        OFF,           // 关闭所有LED
        ANGLE_INDICATOR, // 角度指示器（蓝色灯珠表示角度）
        RANGE_INDICATOR, // 范围指示器（显示角度范围）
        ERROR_INDICATOR, // 错误指示器（红色闪烁）
        BREATHE_EFFECT   // 呼吸灯效果（正弦信号时使用）
    };

    // LED配置结构
    struct LedEffectConfig {
        LedEffect effect_type;
        uint8_t primary_color[3];    // 主颜色 RGB
        uint8_t secondary_color[3];  // 次颜色 RGB
        uint32_t interval_ms;        // 间隔时间（毫秒）
        uint8_t length;              // 效果长度
        bool auto_stop;              // 是否自动停止
    };

    // 角度范围配置
    struct AngleRange {
        float min_angle;  // 最小角度（弧度）
        float max_angle;  // 最大角度（弧度）
        bool is_valid;    // 是否有效
    };

    // 电机角度状态
    struct MotorAngleState {
        float current_angle;     // 当前角度（弧度）
        float target_angle;      // 目标角度（弧度）
        bool is_moving;          // 是否在移动
        bool is_error;           // 是否有错误
        AngleRange valid_range;  // 有效角度范围
    };

    // 状态变化回调函数类型
    using StateChangeCallback = std::function<void(const MotorAngleState&, const LedEffectConfig&)>;

public:
    /**
     * @brief 构造函数
     * @param led_strip LED灯带指针
     * @param deep_motor 电机控制器指针
     */
    DeepMotorLedState(CircularStrip* led_strip, DeepMotor* deep_motor);

    /**
     * @brief 析构函数
     */
    ~DeepMotorLedState();

    /**
     * @brief 设置电机角度状态
     * @param motor_id 电机ID (1-6)
     * @param state 角度状态
     */
    void SetMotorAngleState(uint8_t motor_id, const MotorAngleState& state);

    /**
     * @brief 更新所有电机角度状态
     * @param states 6个电机的角度状态数组
     */
    void UpdateAllMotorStates(const MotorAngleState states[6]);

    /**
     * @brief 设置角度范围
     * @param motor_id 电机ID (1-6)
     * @param min_angle 最小角度（弧度）
     * @param max_angle 最大角度（弧度）
     */
    void SetAngleRange(uint8_t motor_id, float min_angle, float max_angle);

    /**
     * @brief 启用角度指示器
     * @param motor_id 电机ID (1-6)
     * @param enabled 是否启用
     */
    void EnableAngleIndicator(uint8_t motor_id, bool enabled);

    /**
     * @brief 设置状态变化回调
     * @param callback 回调函数
     */
    void SetStateChangeCallback(StateChangeCallback callback);

    /**
     * @brief 停止当前效果
     */
    void StopCurrentEffect();

    /**
     * @brief 获取当前角度状态
     * @param motor_id 电机ID (1-6)
     * @return 角度状态
     */
    MotorAngleState GetMotorAngleState(uint8_t motor_id) const;

    /**
     * @brief 获取角度范围
     * @param motor_id 电机ID (1-6)
     * @return 角度范围
     */
    AngleRange GetAngleRange(uint8_t motor_id) const;

    /**
     * @brief 检查角度指示器是否启用
     * @param motor_id 电机ID (1-6)
     * @return 是否启用
     */
    bool IsAngleIndicatorEnabled(uint8_t motor_id) const;

    /**
     * @brief 启用电机呼吸灯效果（正弦信号时使用）
     * @param motor_id 电机ID (1-6)
     * @param color 呼吸灯颜色
     * @return 是否成功
     */
    bool EnableBreatheEffect(uint8_t motor_id, const StripColor& color);

    /**
     * @brief 禁用电机呼吸灯效果
     * @param motor_id 电机ID (1-6)
     * @return 是否成功
     */
    bool DisableBreatheEffect(uint8_t motor_id);

    /**
     * @brief 检查电机是否在呼吸灯模式
     * @param motor_id 电机ID (1-6)
     * @return 是否在呼吸灯模式
     */
    bool IsBreatheEffectEnabled(uint8_t motor_id) const;

private:
    static constexpr int LED_COUNT = 24;           // LED数量
    static constexpr int MOTOR_COUNT = 6;          // 电机数量（保持兼容性）
    static constexpr float PI = 3.14159265359f;    // 圆周率
    static constexpr float TWO_PI = 2.0f * PI;     // 2π

    CircularStrip* led_strip_;                     // LED灯带指针
    DeepMotor* deep_motor_;                       // 电机控制器指针
    
    // 保持多电机数组以兼容现有接口，但逻辑上只关注活跃电机
    MotorAngleState motor_states_[MOTOR_COUNT];   // 6个电机的角度状态
    AngleRange angle_ranges_[MOTOR_COUNT];        // 6个电机的角度范围
    bool angle_indicators_enabled_[MOTOR_COUNT];  // 6个电机的角度指示器启用状态
    bool breathe_effects_enabled_[MOTOR_COUNT];   // 6个电机的呼吸灯效果启用状态
    StripColor breathe_colors_[MOTOR_COUNT];      // 6个电机的呼吸灯颜色
    
    // LED索引缓存，用于避免不必要的更新
    int last_angle_led_index_;                    // 上次角度对应的LED索引
    int last_target_led_index_;                   // 上次目标角度对应的LED索引
    bool led_indices_initialized_;                // LED索引是否已初始化
    
    StateChangeCallback state_change_callback_;   // 状态变化回调
    LedEffectConfig current_effect_;              // 当前效果配置

    /**
     * @brief 将角度转换为LED索引
     * @param angle 角度（弧度）
     * @return LED索引 (0-23)
     */
    int AngleToLedIndex(float angle) const;

    /**
     * @brief 将LED索引转换为角度
     * @param led_index LED索引 (0-23)
     * @return 角度（弧度）
     */
    float LedIndexToAngle(int led_index) const;

    /**
     * @brief 应用角度指示器效果
     * @param motor_id 电机ID (1-6)
     */
    void ApplyAngleIndicatorEffect(uint8_t motor_id);

    /**
     * @brief 应用范围指示器效果
     * @param motor_id 电机ID (1-6)
     */
    void ApplyRangeIndicatorEffect(uint8_t motor_id);

    /**
     * @brief 应用错误指示器效果
     * @param motor_id 电机ID (1-6)
     */
    void ApplyErrorIndicatorEffect(uint8_t motor_id);

    /**
     * @brief 更新LED显示
     */
    void UpdateLedDisplay();

    /**
     * @brief 状态变化处理
     * @param motor_id 电机ID (1-6)
     * @param new_state 新状态
     */
    void OnStateChanged(uint8_t motor_id, const MotorAngleState& new_state);
};
