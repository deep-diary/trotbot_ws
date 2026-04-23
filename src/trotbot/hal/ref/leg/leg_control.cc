#include "leg_control.h"
#include "../config.h"
#include "motor/deep_motor.h"
#include "motor/protocol_motor.h"
#include "mcp_server.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string>
#include <cstring>

#define TAG "LegControl"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 各腿默认：电机 ID、站立位、行走限位、机械限位（与 dog/README 表一致）
// 行走 = 正常步态规划用 clamp；机械 = 下发前最后一道安全边界
static const struct {
    uint8_t motor_ids[LEG_JOINT_COUNT];
    float stance[LEG_JOINT_COUNT];
    float limit_low[LEG_JOINT_COUNT];
    float limit_high[LEG_JOINT_COUNT];
    float mech_low[LEG_JOINT_COUNT];
    float mech_high[LEG_JOINT_COUNT];
} LEG_DEFAULTS[] = {
    // FL / RL：左前、后左
    { {11, 12, 13}, {DEEP_DOG_STANCE_HIP_AA_RAD, DEEP_DOG_STANCE_HIP_FE_RAD, -DEEP_DOG_STANCE_KNEE_RAD},
      {DEEP_DOG_STANCE_HIP_AA_RAD - LEG_DEFAULT_HIP_AA_AMP,
       DEEP_DOG_STANCE_HIP_FE_RAD - LEG_DEFAULT_HIP_FE_AMP,
       -DEEP_DOG_STANCE_KNEE_RAD - LEG_DEFAULT_KNEE_AMP},
      {DEEP_DOG_STANCE_HIP_AA_RAD + LEG_DEFAULT_HIP_AA_AMP,
       DEEP_DOG_STANCE_HIP_FE_RAD + LEG_DEFAULT_HIP_FE_AMP,
       -DEEP_DOG_STANCE_KNEE_RAD + LEG_DEFAULT_KNEE_AMP},
      {-0.3f, -0.6f, -2.26f}, {0.3f, 1.4f, 0.0f} },
    // FR / RR：右前、后右（髋前后与膝符号与左侧对称）
    { {21, 22, 23}, {DEEP_DOG_STANCE_HIP_AA_RAD, -DEEP_DOG_STANCE_HIP_FE_RAD, DEEP_DOG_STANCE_KNEE_RAD},
      {DEEP_DOG_STANCE_HIP_AA_RAD - LEG_DEFAULT_HIP_AA_AMP,
       -DEEP_DOG_STANCE_HIP_FE_RAD - LEG_DEFAULT_HIP_FE_AMP,
       DEEP_DOG_STANCE_KNEE_RAD - LEG_DEFAULT_KNEE_AMP},
      {DEEP_DOG_STANCE_HIP_AA_RAD + LEG_DEFAULT_HIP_AA_AMP,
       -DEEP_DOG_STANCE_HIP_FE_RAD + LEG_DEFAULT_HIP_FE_AMP,
       DEEP_DOG_STANCE_KNEE_RAD + LEG_DEFAULT_KNEE_AMP},
      {-0.3f, -1.4f, 0.0f}, {0.3f, 0.6f, 2.26f} },
    { {51, 52, 53}, {DEEP_DOG_STANCE_HIP_AA_RAD, DEEP_DOG_STANCE_HIP_FE_RAD, -DEEP_DOG_STANCE_KNEE_RAD},
      {DEEP_DOG_STANCE_HIP_AA_RAD - LEG_DEFAULT_HIP_AA_AMP,
       DEEP_DOG_STANCE_HIP_FE_RAD - LEG_DEFAULT_HIP_FE_AMP,
       -DEEP_DOG_STANCE_KNEE_RAD - LEG_DEFAULT_KNEE_AMP},
      {DEEP_DOG_STANCE_HIP_AA_RAD + LEG_DEFAULT_HIP_AA_AMP,
       DEEP_DOG_STANCE_HIP_FE_RAD + LEG_DEFAULT_HIP_FE_AMP,
       -DEEP_DOG_STANCE_KNEE_RAD + LEG_DEFAULT_KNEE_AMP},
      {-0.3f, -0.6f, -2.26f}, {0.3f, 1.4f, 0.0f} },
    { {61, 62, 63}, {DEEP_DOG_STANCE_HIP_AA_RAD, -DEEP_DOG_STANCE_HIP_FE_RAD, DEEP_DOG_STANCE_KNEE_RAD},
      {DEEP_DOG_STANCE_HIP_AA_RAD - LEG_DEFAULT_HIP_AA_AMP,
       -DEEP_DOG_STANCE_HIP_FE_RAD - LEG_DEFAULT_HIP_FE_AMP,
       DEEP_DOG_STANCE_KNEE_RAD - LEG_DEFAULT_KNEE_AMP},
      {DEEP_DOG_STANCE_HIP_AA_RAD + LEG_DEFAULT_HIP_AA_AMP,
       -DEEP_DOG_STANCE_HIP_FE_RAD + LEG_DEFAULT_HIP_FE_AMP,
       DEEP_DOG_STANCE_KNEE_RAD + LEG_DEFAULT_KNEE_AMP},
      {-0.3f, -1.4f, 0.0f}, {0.3f, 0.6f, 2.26f} },
};

LegControl::LegControl() {
    memset(motor_ids_, 0, sizeof(motor_ids_));
    memset(stance_position_, 0, sizeof(stance_position_));
}

void LegControl::setLegType(LegType type) {
    leg_type_ = type;
    size_t idx = (size_t)type;
    if (idx < 4) {
        memcpy(motor_ids_, LEG_DEFAULTS[idx].motor_ids, sizeof(motor_ids_));
        memcpy(stance_position_, LEG_DEFAULTS[idx].stance, sizeof(stance_position_));
        memcpy(limit_low_, LEG_DEFAULTS[idx].limit_low, sizeof(limit_low_));
        memcpy(limit_high_, LEG_DEFAULTS[idx].limit_high, sizeof(limit_high_));
        memcpy(mech_limit_low_, LEG_DEFAULTS[idx].mech_low, sizeof(mech_limit_low_));
        memcpy(mech_limit_high_, LEG_DEFAULTS[idx].mech_high, sizeof(mech_limit_high_));
    }
    ESP_LOGI(TAG, "setLegType %d, motors %d %d %d", (int)type, motor_ids_[0], motor_ids_[1], motor_ids_[2]);
}

void LegControl::setMotorIds(uint8_t hip_aa, uint8_t hip_fe, uint8_t knee) {
    motor_ids_[LEG_JOINT_HIP_AA] = hip_aa;
    motor_ids_[LEG_JOINT_HIP_FE] = hip_fe;
    motor_ids_[LEG_JOINT_KNEE] = knee;
}

void LegControl::setLimits(const float limit_low[LEG_JOINT_COUNT], const float limit_high[LEG_JOINT_COUNT]) {
    memcpy(limit_low_, limit_low, sizeof(limit_low_));
    memcpy(limit_high_, limit_high, sizeof(limit_high_));
}

void LegControl::setStancePosition(const float stance[LEG_JOINT_COUNT]) {
    memcpy(stance_position_, stance, sizeof(stance_position_));
}

float LegControl::clampJoint(int joint_index, float value) const {
    if (joint_index < 0 || joint_index >= LEG_JOINT_COUNT) return value;
    if (value < limit_low_[joint_index]) return limit_low_[joint_index];
    if (value > limit_high_[joint_index]) return limit_high_[joint_index];
    return value;
}

float LegControl::clampJointMechanical(int joint_index, float value) const {
    if (joint_index < 0 || joint_index >= LEG_JOINT_COUNT) return value;
    if (value < mech_limit_low_[joint_index]) return mech_limit_low_[joint_index];
    if (value > mech_limit_high_[joint_index]) return mech_limit_high_[joint_index];
    return value;
}

void LegControl::clampJointPositionsMechanical(float pos[LEG_JOINT_COUNT]) const {
    for (int j = 0; j < LEG_JOINT_COUNT; j++) {
        float v = pos[j];
        float c = clampJointMechanical(j, v);
        if (c != v) {
            ESP_LOGW(TAG, "机械限位裁剪 腿类型=%d 关节%d: %.4f -> %.4f rad", (int)leg_type_, j, v, c);
        }
        pos[j] = c;
    }
}

void LegControl::computeStepPositionAt(uint16_t step_index, float out_position[LEG_JOINT_COUNT], bool forward) const {
    if (total_steps_ == 0) {
        memcpy(out_position, stance_position_, sizeof(float) * LEG_JOINT_COUNT);
        return;
    }
    float phase = 2.0f * (float)M_PI * (float)step_index / (float)total_steps_;
    float s0 = sinf(phase);
    float c0 = cosf(phase);
    float s = forward ? s0 : -s0;
    float c = forward ? c0 : -c0;
    /* 摆动半周：与前进方向一致的 sin 半周上抬脚（sin² 包络） */
    float swing_phase = forward ? s0 : -s0;
    float swing_bump = (swing_phase > 0.f) ? (swing_phase * swing_phase) : 0.f;

    bool right_leg = (leg_type_ == LegType::FR || leg_type_ == LegType::RR);
    float hip_fe_sign = right_leg ? -1.0f : 1.0f;
    float knee_sign = right_leg ? -1.0f : 1.0f;
    /* 左膝负向站立、弯向 0 为加正；右膝正向站立、弯向 0 为减正 → 抬脚为左 +、右 - */
    float knee_lift_sign = right_leg ? -1.0f : 1.0f;

    out_position[LEG_JOINT_HIP_AA] = clampJoint(LEG_JOINT_HIP_AA, stance_position_[LEG_JOINT_HIP_AA] + hip_aa_amp_ * s);
    out_position[LEG_JOINT_HIP_FE] = clampJoint(LEG_JOINT_HIP_FE,
        stance_position_[LEG_JOINT_HIP_FE] + hip_fe_sign * (hip_fe_amp_ * s + hip_fe_cos_amp_ * c));
    out_position[LEG_JOINT_KNEE] = clampJoint(LEG_JOINT_KNEE,
        stance_position_[LEG_JOINT_KNEE] + knee_sign * knee_amp_ * s + knee_lift_sign * knee_swing_lift_ * swing_bump);
}

void LegControl::computeStepPosition(float out_position[LEG_JOINT_COUNT], bool forward) const {
    computeStepPositionAt(current_step_, out_position, forward);
}

void LegControl::fillStepPositionsAtStepIndex(uint16_t step_index, float out[LEG_JOINT_COUNT], bool forward) const {
    computeStepPositionAt(step_index, out, forward);
}

bool LegControl::init() {
    if (!deep_motor_) {
        ESP_LOGE(TAG, "DeepMotor not set");
        return false;
    }
    for (int i = 0; i < LEG_JOINT_COUNT; i++) {
        uint8_t id = motor_ids_[i];
        if (id == 0) continue;
        if (!deep_motor_->isMotorRegistered(id)) {
            if (!deep_motor_->registerMotor(id)) {
                ESP_LOGE(TAG, "registerMotor fail id=%d (可能已满，最大 %d 个电机)", id, MAX_MOTOR_COUNT);
                return false;
            }
        }
        if (!deep_motor_->initializeMotor(id, 0.0f)) {
            ESP_LOGE(TAG, "deep_motor initializeMotor fail id=%d", id);
            return false;
        }
    }
#if DEEP_DOG_USE_MIT_WALK
    ESP_LOGI(TAG, "leg init ok type=%d (MIT)", (int)leg_type_);
#else
    ESP_LOGI(TAG, "leg init ok type=%d (位置模式)", (int)leg_type_);
#endif
    return true;
}

bool LegControl::disable() {
    for (int i = 0; i < LEG_JOINT_COUNT; i++) {
        if (motor_ids_[i] == 0) {
            continue;
        }
        if (!MotorProtocol::resetMotor(motor_ids_[i])) {
            ESP_LOGE(TAG, "resetMotor fail id=%d", motor_ids_[i]);
            return false;
        }
        if (deep_motor_) {
            deep_motor_->invalidateMotorCommandCache(motor_ids_[i]);
        }
    }
    return true;
}

uint8_t LegControl::getMotorId(int joint_index) const {
    if (joint_index < 0 || joint_index >= LEG_JOINT_COUNT) {
        return 0;
    }
    return motor_ids_[joint_index];
}

float LegControl::getStanceTargetJoint(int joint_index) const {
    if (joint_index < 0 || joint_index >= LEG_JOINT_COUNT) {
        return 0.0f;
    }
    return clampJoint(joint_index, stance_position_[joint_index]);
}

void LegControl::fillCurrentStepPositions(float out[LEG_JOINT_COUNT], bool forward) const {
    computeStepPosition(out, forward);
}

void LegControl::advanceStepForward() {
    current_step_ = (current_step_ + 1) % total_steps_;
}

void LegControl::advanceStepBackward() {
    current_step_ = (current_step_ == 0) ? total_steps_ - 1 : current_step_ - 1;
}

bool LegControl::sendThreeJointsPositionOrMit(const float pos[LEG_JOINT_COUNT], float target_velocity_rad_s) {
    if (!deep_motor_) {
        return false;
    }
#if !DEEP_DOG_USE_MIT_WALK
    for (int i = 0; i < LEG_JOINT_COUNT; i++) {
        if (motor_ids_[i] != 0 && !deep_motor_->setMotorSpeedLimit(motor_ids_[i], target_velocity_rad_s)) {
            ESP_LOGE(TAG, "setMotorSpeedLimit fail id=%d", motor_ids_[i]);
            return false;
        }
    }
#endif
    for (int i = 0; i < LEG_JOINT_COUNT; i++) {
        if (motor_ids_[i] == 0) {
            continue;
        }
#if DEEP_DOG_USE_MIT_WALK
        if (!deep_motor_->setMotorMitCommand(motor_ids_[i], pos[i], DEEP_DOG_MIT_VDES_RAD_S, DEEP_DOG_MIT_DEFAULT_KP,
                                             DEEP_DOG_MIT_DEFAULT_KD, DEEP_DOG_MIT_DEFAULT_TAU_FF)) {
            ESP_LOGE(TAG, "setMotorMitCommand fail id=%d", motor_ids_[i]);
            return false;
        }
#else
        if (!deep_motor_->setMotorPositionRefOnly(motor_ids_[i], pos[i])) {
            ESP_LOGE(TAG, "setMotorPositionRefOnly fail id=%d", motor_ids_[i]);
            return false;
        }
#endif
    }
    return true;
}

bool LegControl::goToZero(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    float pos[LEG_JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
    clampJointPositionsMechanical(pos);
#if DEEP_DOG_USE_MIT_WALK
    constexpr bool kEnableZeroHold = true;
    constexpr int kZeroHoldMs = 200;
#endif
    if (!sendThreeJointsPositionOrMit(pos, max_speed_rad_s)) {
        return false;
    }
#if DEEP_DOG_USE_MIT_WALK
    if (kEnableZeroHold) {
        vTaskDelay(pdMS_TO_TICKS(kZeroHoldMs));
        if (!sendThreeJointsPositionOrMit(pos, max_speed_rad_s)) {
            return false;
        }
    }
#endif
    return true;
}

bool LegControl::goToStance(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    float pos[LEG_JOINT_COUNT];
    for (int i = 0; i < LEG_JOINT_COUNT; i++) {
        pos[i] = clampJoint(i, stance_position_[i]);
    }
    clampJointPositionsMechanical(pos);
    return sendThreeJointsPositionOrMit(pos, max_speed_rad_s);
}

bool LegControl::stepForward(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    advanceStepForward();
    float pos[LEG_JOINT_COUNT];
    fillCurrentStepPositions(pos, true);
    clampJointPositionsMechanical(pos);
    return sendThreeJointsPositionOrMit(pos, max_speed_rad_s);
}

bool LegControl::stepBackward(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    advanceStepBackward();
    float pos[LEG_JOINT_COUNT];
    fillCurrentStepPositions(pos, false);
    clampJointPositionsMechanical(pos);
    return sendThreeJointsPositionOrMit(pos, max_speed_rad_s);
}

// --- MCP 工具注册 ---
// leg_id 支持多种说法：英文 fl/fr/rl/rr，中文 前左/前右/后左/后右 或 左前/右前/左后/右后（可带「腿」），编号 1~4 或 1号腿…
static int str_to_leg_index(const std::string& s) {
    if (s.empty()) return -1;
    // 英文缩写（小写）
    if (s == "fl") return 0;
    if (s == "fr") return 1;
    if (s == "rl") return 2;
    if (s == "rr") return 3;
    // 中文：前左/前左腿、前右/前右腿、后左/后左腿、后右/后右腿
    if (s == "前左" || s == "前左腿") return 0;
    if (s == "前右" || s == "前右腿") return 1;
    if (s == "后左" || s == "后左腿") return 2;
    if (s == "后右" || s == "后右腿") return 3;
    // 中文调换说法：左前/左前腿、右前/右前腿、左后/左后腿、右后/右后腿
    if (s == "左前" || s == "左前腿") return 0;
    if (s == "右前" || s == "右前腿") return 1;
    if (s == "左后" || s == "左后腿") return 2;
    if (s == "右后" || s == "右后腿") return 3;
    // 编号：1号腿=前左 2号腿=前右 3号腿=后左 4号腿=后右
    if (s == "1" || s == "1号" || s == "1号腿") return 0;
    if (s == "2" || s == "2号" || s == "2号腿") return 1;
    if (s == "3" || s == "3号" || s == "3号腿") return 2;
    if (s == "4" || s == "4号" || s == "4号腿") return 3;
    return -1;
}

static const char* leg_index_to_name(int idx) {
    switch (idx) { case 0: return "前左"; case 1: return "前右"; case 2: return "后左"; case 3: return "后右"; default: return "?"; }
}

// 单腿 MCP 的 leg_id 说明（供 LLM/语音填参）：支持 前左/前右/后左/后右 或 左前/右前/左后/右后（可带「腿」），或 1~4 号腿
#define LEG_ID_DESC "leg_id：腿标识。支持 前左/前右/后左/后右 或 左前/右前/左后/右后（可带「腿」），或 1号腿/2号腿/3号腿/4号腿（1=前左 2=前右 3=后左 4=后右）"

void RegisterLegMcpTools(McpServer& mcp_server, LegControl* legs[4]) {
    mcp_server.AddTool("self.leg.init", "单腿初始化（使能该腿3个电机）。用户说：初始化前左腿、前右腿初始化、初始化1号腿 等时调用。参数 " LEG_ID_DESC, PropertyList(std::vector<Property>{
        Property("leg_id", kPropertyTypeString, std::string("前左"))
    }), [legs](const PropertyList& properties) -> ReturnValue {
        std::string lid = properties["leg_id"].value<std::string>();
        int idx = str_to_leg_index(lid);
        if (idx < 0 || !legs[idx]) return std::string("无效 leg_id 或该腿未创建，请使用：前左/前右/后左/后右 或 1号腿/2号腿/3号腿/4号腿");
        if (legs[idx]->init()) return std::string(leg_index_to_name(idx)) + std::string("腿 初始化成功");
        return std::string(leg_index_to_name(idx)) + std::string("腿 初始化失败");
    });

    mcp_server.AddTool("self.leg.stand", "单腿站立/单腿站起/回站立位。用户说：前左腿站立、前右腿站起来、2号腿站起 等时调用。参数 " LEG_ID_DESC, PropertyList(std::vector<Property>{
        Property("leg_id", kPropertyTypeString, std::string("前左"))
    }), [legs](const PropertyList& properties) -> ReturnValue {
        std::string lid = properties["leg_id"].value<std::string>();
        int idx = str_to_leg_index(lid);
        if (idx < 0 || !legs[idx]) return std::string("无效 leg_id 或该腿未创建");
        if (legs[idx]->goToStance()) return std::string(leg_index_to_name(idx)) + std::string("腿 已站立");
        return std::string(leg_index_to_name(idx)) + std::string("腿 站立失败");
    });

    mcp_server.AddTool("self.leg.lie_down", "单腿卧倒/趴下/蹲下/回零位。用户说：前左腿卧倒、前右腿蹲下、3号腿趴下、回零位 等时调用（零位=机械零位，不是灵位）。参数 " LEG_ID_DESC, PropertyList(std::vector<Property>{
        Property("leg_id", kPropertyTypeString, std::string("前左"))
    }), [legs](const PropertyList& properties) -> ReturnValue {
        std::string lid = properties["leg_id"].value<std::string>();
        int idx = str_to_leg_index(lid);
        if (idx < 0 || !legs[idx]) return std::string("无效 leg_id 或该腿未创建");
        if (legs[idx]->goToZero()) return std::string(leg_index_to_name(idx)) + std::string("腿 已卧倒");
        return std::string(leg_index_to_name(idx)) + std::string("腿 卧倒失败");
    });

    mcp_server.AddTool("self.leg.step_forward", "单腿向前迈一步。用户说：前左腿向前迈一步、2号腿迈一步 等时调用。参数 " LEG_ID_DESC, PropertyList(std::vector<Property>{
        Property("leg_id", kPropertyTypeString, std::string("前左"))
    }), [legs](const PropertyList& properties) -> ReturnValue {
        std::string lid = properties["leg_id"].value<std::string>();
        int idx = str_to_leg_index(lid);
        if (idx < 0 || !legs[idx]) return std::string("无效 leg_id 或该腿未创建");
        if (legs[idx]->stepForward()) return std::string(leg_index_to_name(idx)) + std::string("腿 向前迈一步成功");
        return std::string(leg_index_to_name(idx)) + std::string("腿 向前迈一步失败");
    });

    mcp_server.AddTool("self.leg.step_back", "单腿向后迈一步。用户说：前左腿向后迈一步 等时调用。参数 " LEG_ID_DESC, PropertyList(std::vector<Property>{
        Property("leg_id", kPropertyTypeString, std::string("前左"))
    }), [legs](const PropertyList& properties) -> ReturnValue {
        std::string lid = properties["leg_id"].value<std::string>();
        int idx = str_to_leg_index(lid);
        if (idx < 0 || !legs[idx]) return std::string("无效 leg_id 或该腿未创建");
        if (legs[idx]->stepBackward()) return std::string(leg_index_to_name(idx)) + std::string("腿 向后迈一步成功");
        return std::string(leg_index_to_name(idx)) + std::string("腿 向后迈一步失败");
    });

    ESP_LOGI(TAG, "Leg MCP tools registered: self.leg.init, stand, lie_down, step_forward, step_back");
}
