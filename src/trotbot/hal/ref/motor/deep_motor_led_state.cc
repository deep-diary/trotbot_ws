#include "deep_motor_led_state.h"
#include "deep_motor.h"  // 需要完整定义以调用getActiveMotorId()
#include "esp_log.h"
#include <cmath>

#define TAG "DeepMotorLedState"

DeepMotorLedState::DeepMotorLedState(CircularStrip* led_strip, DeepMotor* deep_motor) 
    : led_strip_(led_strip), deep_motor_(deep_motor) {
    
    ESP_LOGI(TAG, "电机角度LED状态管理器初始化");
    
    // 初始化所有电机状态
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_states_[i] = {0.0f, 0.0f, false, false, {0.0f, 0.0f, false}};
        angle_ranges_[i] = {0.0f, 0.0f, false};
        angle_indicators_enabled_[i] = false;
        breathe_effects_enabled_[i] = false;
        breathe_colors_[i] = {0, 0, 0};
    }
    
    ESP_LOGI(TAG, "LED状态管理器已简化：只显示当前活跃电机的角度");
    
    // 设置默认效果配置
    current_effect_ = {LedEffect::OFF, {0, 0, 0}, {0, 0, 0}, 0, 0, false};
    
    // 初始化LED索引缓存
    last_angle_led_index_ = -1;
    last_target_led_index_ = -1;
    led_indices_initialized_ = false;
    
    ESP_LOGI(TAG, "电机角度LED状态管理器初始化完成，支持%d个电机，%d颗LED", MOTOR_COUNT, LED_COUNT);
}

DeepMotorLedState::~DeepMotorLedState() {
    StopCurrentEffect();
}

void DeepMotorLedState::SetMotorAngleState(uint8_t motor_id, const MotorAngleState& state) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        ESP_LOGW(TAG, "无效的电机ID: %d", motor_id);
        return;
    }
    
    int index = motor_id - 1;
    MotorAngleState old_state = motor_states_[index];
    motor_states_[index] = state;
    
    // 添加调试日志
    ESP_LOGI(TAG, "SetMotorAngleState: 电机%d, 旧角度=%.3f rad, 新角度=%.3f rad, 角度变化=%s", 
             motor_id, old_state.current_angle, state.current_angle,
             (old_state.current_angle != state.current_angle) ? "是" : "否");
    
    // 检查状态是否发生变化
    if (old_state.current_angle != state.current_angle || 
        old_state.is_moving != state.is_moving || 
        old_state.is_error != state.is_error) {
        ESP_LOGI(TAG, "电机%d状态发生变化，调用OnStateChanged", motor_id);
        OnStateChanged(motor_id, state);
    } else {
        ESP_LOGD(TAG, "电机%d状态无变化，跳过更新", motor_id);
    }
}

void DeepMotorLedState::UpdateAllMotorStates(const MotorAngleState states[6]) {
    bool any_changed = false;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorAngleState old_state = motor_states_[i];
        motor_states_[i] = states[i];
        
        // 检查状态是否发生变化
        if (old_state.current_angle != states[i].current_angle || 
            old_state.is_moving != states[i].is_moving || 
            old_state.is_error != states[i].is_error) {
            any_changed = true;
        }
    }
    
    if (any_changed) {
        UpdateLedDisplay();
    }
}

void DeepMotorLedState::SetAngleRange(uint8_t motor_id, float min_angle, float max_angle) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        ESP_LOGW(TAG, "无效的电机ID: %d", motor_id);
        return;
    }
    
    int index = motor_id - 1;
    angle_ranges_[index] = {min_angle, max_angle, true};
    
    ESP_LOGI(TAG, "设置电机%d角度范围: [%.2f, %.2f] rad", motor_id, min_angle, max_angle);
}

void DeepMotorLedState::EnableAngleIndicator(uint8_t motor_id, bool enabled) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        ESP_LOGW(TAG, "无效的电机ID: %d", motor_id);
        return;
    }
    
    int index = motor_id - 1;
    angle_indicators_enabled_[index] = enabled;
    
    ESP_LOGI(TAG, "电机%d角度指示器: %s", motor_id, enabled ? "启用" : "禁用");
    
    if (enabled) {
        UpdateLedDisplay();
    } else {
        StopCurrentEffect();
    }
}

void DeepMotorLedState::SetStateChangeCallback(StateChangeCallback callback) {
    state_change_callback_ = callback;
}

void DeepMotorLedState::StopCurrentEffect() {
    if (led_strip_) {
        led_strip_->SetAllColor({0, 0, 0});  // 关闭所有LED
    }
    current_effect_ = {LedEffect::OFF, {0, 0, 0}, {0, 0, 0}, 0, 0, false};
}

DeepMotorLedState::MotorAngleState DeepMotorLedState::GetMotorAngleState(uint8_t motor_id) const {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        return {0.0f, 0.0f, false, true, {0.0f, 0.0f, false}};
    }
    
    return motor_states_[motor_id - 1];
}

DeepMotorLedState::AngleRange DeepMotorLedState::GetAngleRange(uint8_t motor_id) const {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        return {0.0f, 0.0f, false};
    }
    
    return angle_ranges_[motor_id - 1];
}

bool DeepMotorLedState::IsAngleIndicatorEnabled(uint8_t motor_id) const {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        return false;
    }
    
    return angle_indicators_enabled_[motor_id - 1];
}

int DeepMotorLedState::AngleToLedIndex(float angle) const {
    float original_angle = angle;
    
    // 将角度标准化到 [0, 2π) 范围
    while (angle < 0) angle += TWO_PI;
    while (angle >= TWO_PI) angle -= TWO_PI;
    
    // 将角度映射到LED索引 (0-23)
    // 0度对应第0个LED，180度对应第12个LED
    int led_index = static_cast<int>((angle / TWO_PI) * LED_COUNT);
    
    // 确保索引在有效范围内
    if (led_index >= LED_COUNT) led_index = LED_COUNT - 1;
    if (led_index < 0) led_index = 0;
    
    // 添加调试日志
    ESP_LOGI(TAG, "AngleToLedIndex: 原始角度=%.3f rad (%.1f°), 标准化角度=%.3f rad (%.1f°), LED索引=%d", 
             original_angle, original_angle * 180.0f / 3.14159f,
             angle, angle * 180.0f / 3.14159f, led_index);
    
    return led_index;
}

float DeepMotorLedState::LedIndexToAngle(int led_index) const {
    if (led_index < 0 || led_index >= LED_COUNT) {
        return 0.0f;
    }
    
    // 将LED索引转换为角度
    return (static_cast<float>(led_index) / LED_COUNT) * TWO_PI;
}

void DeepMotorLedState::ApplyAngleIndicatorEffect(uint8_t motor_id) {
    if (!led_strip_) {
        ESP_LOGW(TAG, "LED灯带指针为空，无法应用角度指示器效果");
        return;
    }
    
    int index = motor_id - 1;
    const MotorAngleState& state = motor_states_[index];
    
    // 注意：不在这里关闭所有LED，由UpdateLedDisplay统一管理
    
    // 计算当前角度对应的LED索引
    int led_index = AngleToLedIndex(state.current_angle);
    
    // 添加详细调试日志
    ESP_LOGI(TAG, "ApplyAngleIndicatorEffect: 电机%d, 角度=%.3f rad (%.1f°), LED索引=%d", 
             motor_id, state.current_angle, state.current_angle * 180.0f / 3.14159f, led_index);
    
    // 设置当前角度位置的LED为蓝色
    StripColor blue_color = {0, 0, 255};
    led_strip_->SetSingleColor(led_index, blue_color);
    ESP_LOGI(TAG, "设置LED%d为蓝色 (RGB: 0,0,255)", led_index);
    
    // 如果电机正在移动，目标角度用青色表示
    if (state.is_moving) {
        // 获取真实的目标位置（从DeepMotor获取）
        float real_target_angle;
        if (deep_motor_->getMotorTargetAngle(motor_id, &real_target_angle)) {
            int target_led_index = AngleToLedIndex(real_target_angle);
            StripColor cyan_color = {0, 255, 255};
            led_strip_->SetSingleColor(target_led_index, cyan_color);
            ESP_LOGI(TAG, "设置目标LED%d为青色 (RGB: 0,255,255), 目标角度=%.3f rad", 
                     target_led_index, real_target_angle);
        }
    }
    
    // 如果有有效角度范围，用绿色表示范围边界
    if (angle_ranges_[index].is_valid) {
        int min_led_index = AngleToLedIndex(angle_ranges_[index].min_angle);
        int max_led_index = AngleToLedIndex(angle_ranges_[index].max_angle);
        StripColor green_color = {0, 255, 0};
        
        led_strip_->SetSingleColor(min_led_index, green_color);
        led_strip_->SetSingleColor(max_led_index, green_color);
    }
    
    ESP_LOGD(TAG, "电机%d角度指示器: 当前角度%.2f rad -> LED%d", 
             motor_id, state.current_angle, led_index);
}

void DeepMotorLedState::ApplyRangeIndicatorEffect(uint8_t motor_id) {
    if (!led_strip_) {
        ESP_LOGW(TAG, "LED灯带指针为空，无法应用范围指示器效果");
        return;
    }
    
    int index = motor_id - 1;
    const AngleRange& range = angle_ranges_[index];
    
    if (!range.is_valid) {
        ESP_LOGW(TAG, "电机%d没有有效的角度范围", motor_id);
        return;
    }
    
    // 关闭所有LED
    led_strip_->SetAllColor({0, 0, 0});
    
    // 计算范围边界对应的LED索引
    int min_led_index = AngleToLedIndex(range.min_angle);
    int max_led_index = AngleToLedIndex(range.max_angle);
    
    // 用绿色表示有效角度范围
    StripColor green_color = {0, 255, 0};
    
    // 如果范围跨越0度，需要特殊处理
    if (min_led_index > max_led_index) {
        // 范围跨越0度，分别设置两段
        for (int i = 0; i <= max_led_index; i++) {
            led_strip_->SetSingleColor(i, green_color);
        }
        for (int i = min_led_index; i < LED_COUNT; i++) {
            led_strip_->SetSingleColor(i, green_color);
        }
    } else {
        // 正常范围，设置连续段
        for (int i = min_led_index; i <= max_led_index; i++) {
            led_strip_->SetSingleColor(i, green_color);
        }
    }
    
    ESP_LOGD(TAG, "电机%d范围指示器: [%.2f, %.2f] rad -> LED[%d, %d]", 
             motor_id, range.min_angle, range.max_angle, min_led_index, max_led_index);
}

void DeepMotorLedState::ApplyErrorIndicatorEffect(uint8_t motor_id) {
    if (!led_strip_) {
        ESP_LOGW(TAG, "LED灯带指针为空，无法应用错误指示器效果");
        return;
    }
    
    // 红色闪烁表示错误
    StripColor red_color = {255, 0, 0};
    led_strip_->Blink(red_color, 200);  // 200ms间隔闪烁
    
    ESP_LOGD(TAG, "电机%d错误指示器: 红色闪烁", motor_id);
}

void DeepMotorLedState::UpdateLedDisplay() {
    if (!led_strip_) {
        return;
    }
    
    // 简化逻辑：只显示当前活跃电机
    if (!deep_motor_) {
        ESP_LOGW(TAG, "DeepMotor指针为空，无法获取活跃电机");
        return;
    }
    
    int8_t active_motor_id = deep_motor_->getActiveMotorId();
    if (active_motor_id <= 0) {
        ESP_LOGD(TAG, "没有活跃电机，关闭所有LED");
        led_strip_->SetAllColor({0, 0, 0});
        // 重置LED索引缓存
        last_angle_led_index_ = -1;
        last_target_led_index_ = -1;
        led_indices_initialized_ = false;
        return;
    }
    
    int motor_index = active_motor_id - 1;
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        ESP_LOGW(TAG, "活跃电机ID %d 超出范围", active_motor_id);
        return;
    }
    
    // 检查活跃电机的角度指示器是否启用
    if (!angle_indicators_enabled_[motor_index]) {
        ESP_LOGD(TAG, "活跃电机%d的角度指示器未启用", active_motor_id);
        led_strip_->SetAllColor({0, 0, 0});
        return;
    }
    
    // 检查是否在呼吸灯模式
    if (breathe_effects_enabled_[motor_index]) {
        ESP_LOGD(TAG, "活跃电机%d在呼吸灯模式，跳过角度显示", active_motor_id);
        return;
    }
    
    const MotorAngleState& state = motor_states_[motor_index];
    
    // 计算当前角度和目标角度对应的LED索引
    int current_led_index = AngleToLedIndex(state.current_angle);
    int target_led_index = -1;
    
    // 获取真实的目标位置（从DeepMotor获取）
    float real_target_angle;
    if (deep_motor_->getMotorTargetAngle(active_motor_id, &real_target_angle)) {
        target_led_index = AngleToLedIndex(real_target_angle);
    }
    
    // 检查LED索引是否有变化，如果没有变化则跳过更新
    if (led_indices_initialized_ && 
        current_led_index == last_angle_led_index_ && 
        target_led_index == last_target_led_index_ &&
        !state.is_error) {
        ESP_LOGD(TAG, "LED索引无变化，跳过更新: 角度LED=%d, 目标LED=%d", 
                 current_led_index, target_led_index);
        return;
    }
    
    ESP_LOGI(TAG, "UpdateLedDisplay: 活跃电机%d, 角度=%.3f rad (%.1f°), 错误=%s", 
             active_motor_id, state.current_angle, state.current_angle * 180.0f / 3.14159f,
             state.is_error ? "是" : "否");
    
    // 更新LED索引缓存
    last_angle_led_index_ = current_led_index;
    last_target_led_index_ = target_led_index;
    led_indices_initialized_ = true;
    
    // 关闭所有LED
    led_strip_->SetAllColor({0, 0, 0});
    
    if (state.is_error) {
        // 错误状态用红色闪烁
        ESP_LOGI(TAG, "活跃电机%d处于错误状态，应用错误指示器", active_motor_id);
        ApplyErrorIndicatorEffect(active_motor_id);
    } else {
        // 正常状态显示角度
        ESP_LOGI(TAG, "活跃电机%d正常状态，应用角度指示器", active_motor_id);
        ApplyAngleIndicatorEffect(active_motor_id);
    }
}

void DeepMotorLedState::OnStateChanged(uint8_t motor_id, const MotorAngleState& new_state) {
    ESP_LOGD(TAG, "电机%d状态变化: 角度%.2f rad, 移动:%s, 错误:%s", 
             motor_id, new_state.current_angle, 
             new_state.is_moving ? "是" : "否",
             new_state.is_error ? "是" : "否");
    
    // 更新LED显示
    UpdateLedDisplay();
    
    // 调用状态变化回调
    if (state_change_callback_) {
        state_change_callback_(new_state, current_effect_);
    }
}

// ========== 呼吸灯效果相关方法 ==========

bool DeepMotorLedState::EnableBreatheEffect(uint8_t motor_id, const StripColor& color) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        ESP_LOGW(TAG, "无效的电机ID: %d", motor_id);
        return false;
    }
    
    if (!led_strip_) {
        ESP_LOGW(TAG, "LED灯带指针为空，无法启用呼吸灯效果");
        return false;
    }
    
    int index = motor_id - 1;
    breathe_effects_enabled_[index] = true;
    breathe_colors_[index] = color;
    
    // 启动呼吸灯效果
    led_strip_->Breathe(color, {0, 0, 0}, 50); // 1秒呼吸周期
    
    ESP_LOGI(TAG, "启用电机%d呼吸灯效果，颜色: RGB(%d,%d,%d)", 
             motor_id, color.red, color.green, color.blue);
    
    return true;
}

bool DeepMotorLedState::DisableBreatheEffect(uint8_t motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        ESP_LOGW(TAG, "无效的电机ID: %d", motor_id);
        return false;
    }
    
    int index = motor_id - 1;
    breathe_effects_enabled_[index] = false;
    
    // 停止呼吸灯效果，切换回角度指示器
    if (angle_indicators_enabled_[index]) {
        UpdateLedDisplay();
    } else {
        // 如果没有启用角度指示器，关闭所有LED
        if (led_strip_) {
            led_strip_->SetAllColor({0, 0, 0});
        }
    }
    
    ESP_LOGI(TAG, "禁用电机%d呼吸灯效果，切换回角度指示器", motor_id);
    
    return true;
}

bool DeepMotorLedState::IsBreatheEffectEnabled(uint8_t motor_id) const {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        return false;
    }
    
    int index = motor_id - 1;
    return breathe_effects_enabled_[index];
}
