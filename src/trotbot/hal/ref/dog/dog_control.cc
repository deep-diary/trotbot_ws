#include "dog_control.h"
#include "config.h"
#include "motor/deep_motor.h"
#include "mcp_server.h"
#include "trajectory/trajectory_planner.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

static bool IsContinuousWalkingPose(DogPoseState s) {
    return s == DogPoseState::WalkingForward || s == DogPoseState::WalkingBackward;
}

#define TAG "DogControl"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float dog_rad_to_deg(float rad) {
    return rad * 180.0f / (float)M_PI;
}

static int ClampInt(int v, int lo, int hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float ClampFloat(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static trajectory_planner_t* AcquirePosePlanner() {
    static trajectory_planner_t* planner = nullptr;
    if (planner) {
        return planner;
    }
    planner = static_cast<trajectory_planner_t*>(
        heap_caps_malloc(sizeof(trajectory_planner_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!planner) {
        planner = static_cast<trajectory_planner_t*>(heap_caps_malloc(sizeof(trajectory_planner_t), MALLOC_CAP_8BIT));
    }
    return planner;
}

bool DogControl::moveToPoseJointsInterp(const char* label, const float target[4][LEG_JOINT_COUNT], float max_speed_rad_s) {
    if (!deep_motor_) return false;
    stopContinuousLocomotionIfNeeded();
    gait_planner_.resetCycle();
    for (int i = 0; i < 4; i++) {
        legs_[i].setCurrentStep(gait_planner_.effectiveStepForLeg(i));
    }

    float pos[4][LEG_JOINT_COUNT] = {};
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            pos[leg][j] = target[leg][j];
        }
    }
    clampLegsMechanical(pos);
    logJointTargetsBeforeSend(label ? label : "pose", pos);

    float start_flat[TRAJECTORY_MAX_MOTOR_COUNT] = {0};
    float end_flat[TRAJECTORY_MAX_MOTOR_COUNT] = {0};
    int idx = 0;
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            float rad = 0.0f;
            const uint8_t id = legs_[leg].getMotorId(j);
            if (id != 0) {
                if (!deep_motor_->getMotorActualPosition(id, &rad)) {
                    rad = 0.0f;
                }
            }
            start_flat[idx] = rad;
            end_flat[idx] = pos[leg][j];
            idx++;
        }
    }

    trajectory_planner_t* planner = AcquirePosePlanner();
    if (!planner) {
        ESP_LOGE(TAG, "%s: pose planner alloc failed", label ? label : "pose");
        return false;
    }
    if (!trajectory_plan_linear_fixed_duration(planner, start_flat, end_flat, 12, DEEP_DOG_POSE_INTERP_POINTS,
                                               DEEP_DOG_POSE_INTERP_DURATION_MS)) {
        return false;
    }

    trajectory_point_t p;
    uint32_t last_t = 0;
    while (trajectory_get_next_point(planner, &p)) {
        float interp[4][LEG_JOINT_COUNT] = {};
        int k = 0;
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < LEG_JOINT_COUNT; j++) {
                interp[leg][j] = p.positions[k++];
            }
        }
        clampLegsMechanical(interp);
        if (!sendAllLegJointTargets(interp, max_speed_rad_s)) {
            return false;
        }
        const uint32_t dt_ms = p.time_ms - last_t;
        last_t = p.time_ms;
        if (dt_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(dt_ms));
        }
    }
    // 末端保持优先使用“最后目标点”，避免读取反馈滞后造成回拉/抖动。
    if (!sendHoldCurrentPoseZeroSpeed(label ? label : "pose_done", pos)) {
        return false;
    }
    return true;
}

bool DogControl::goToStaticPose(DogStaticPoseId pose, float max_speed_rad_s) {
    float target[4][LEG_JOINT_COUNT] = {};
    FillDogStaticPose(pose, legs_, target);
    return moveToPoseJointsInterp(DogStaticPoseName(pose), target, max_speed_rad_s);
}

DogControl::DogControl() {
    legs_[0].setLegType(LegType::FL);
    legs_[1].setLegType(LegType::FR);
    legs_[2].setLegType(LegType::RL);
    legs_[3].setLegType(LegType::RR);

#if DEEP_DOG_MIT_VALIDATE_FL_ONLY
    mit_fl_only_ = true;
#endif

    gait_planner_.setTotalSteps(LEG_DEFAULT_TOTAL_STEPS);
    for (int i = 0; i < 4; i++) {
        legs_[i].setTotalSteps(LEG_DEFAULT_TOTAL_STEPS);
    }
}

void DogControl::setDeepMotor(DeepMotor* motor) {
    deep_motor_ = motor;
    for (int i = 0; i < 4; i++) {
        legs_[i].setDeepMotor(motor);
    }
}

void DogControl::getLegs(LegControl* out_legs[4]) {
    for (int i = 0; i < 4; i++) {
        out_legs[i] = &legs_[i];
    }
}

void DogControl::setGaitTotalSteps(uint16_t steps) {
    const uint16_t old_steps = gait_planner_.getTotalSteps();
    const int old_cycle_ms = (old_steps > 0) ? (continuous_step_period_ms_ * (int)old_steps)
                                             : DEEP_DOG_CYCLE_PERIOD_MS_DEFAULT;
    if (steps < 4) {
        steps = 4;
    }
    gait_planner_.setTotalSteps(steps);
    for (int i = 0; i < 4; i++) {
        legs_[i].setTotalSteps(steps);
        legs_[i].setCurrentStep(gait_planner_.effectiveStepForLeg(i));
    }
    // 保持整步周期不变，避免仅改采样点数导致体感速度突变。
    setContinuousCyclePeriodMs(old_cycle_ms);
    ESP_LOGI(TAG, "步态周期采样点数已更新为 %u", (unsigned)steps);
}

void DogControl::clampLegsMechanical(float pos[4][LEG_JOINT_COUNT]) {
    for (int leg = 0; leg < 4; leg++) {
        legs_[leg].clampJointPositionsMechanical(pos[leg]);
    }
}

bool DogControl::ensureStandingForWalk(float max_speed_rad_s) {
    if (!deep_motor_) {
        return false;
    }
    if (state_machine_.state() == DogPoseState::Uninitialized) {
        ESP_LOGW(TAG, "行走被拒绝：请先执行整机初始化 (self.dog.init)");
        return false;
    }
    if (IsContinuousWalkingPose(state_machine_.state())) {
        return true;
    }
    if (!state_machine_.needsStandBeforeWalk()) {
        return true;
    }
    ESP_LOGI(TAG, "当前为趴下或姿态未确认，先站立再行走");
    return stand(max_speed_rad_s);
}

void DogControl::logJointTargetsBeforeSend(const char* motion_label, const float pos[4][LEG_JOINT_COUNT]) const {
    static const char* leg_names[] = {"FL", "FR", "RL", "RR"};
    static const char* joint_names[] = {"HipAA", "HipFE", "Knee"};
    ESP_LOGD(TAG, "[关节目标] %s (rad / deg)", motion_label);
    for (int leg = 0; leg < 4; leg++) {
        ESP_LOGD(TAG, "  腿%s: %s=%.4f(%.1f°)  %s=%.4f(%.1f°)  %s=%.4f(%.1f°)",
                 leg_names[leg],
                 joint_names[0], pos[leg][0], dog_rad_to_deg(pos[leg][0]),
                 joint_names[1], pos[leg][1], dog_rad_to_deg(pos[leg][1]),
                 joint_names[2], pos[leg][2], dog_rad_to_deg(pos[leg][2]));
    }
}

bool DogControl::sendAllLegJointTargets(const float pos[4][LEG_JOINT_COUNT], float max_speed_rad_s) {
    if (!deep_motor_) {
        return false;
    }
    // 缓存“上一帧目标”，便于调 MIT 增益后立即重发（只改 kp/kd，其它不变）。
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            last_joint_targets_[leg][j] = pos[leg][j];
        }
    }
    last_max_speed_rad_s_ = max_speed_rad_s;
    has_last_joint_targets_ = true;

    for (int j = 0; j < LEG_JOINT_COUNT; j++) {
        for (int leg = 0; leg < 4; leg++) {
            if (mit_fl_only_ && leg != 0) {
                continue;
            }
            uint8_t id = legs_[leg].getMotorId(j);
            if (id == 0) {
                continue;
            }
            float pj = pos[leg][j];
#if DEEP_DOG_USE_MIT_WALK
            if (!deep_motor_->setMotorMitCommand(id, pj, DEEP_DOG_MIT_VDES_RAD_S, mit_kp_, mit_kd_, mit_tau_ff_)) {
                ESP_LOGE(TAG, "sendAllLegJointTargets setMotorMitCommand leg=%d joint=%d id=%u", leg, j,
                         (unsigned)id);
                return false;
            }
#else
            if (!deep_motor_->setMotorPositionRefOnly(id, pj)) {
                ESP_LOGE(TAG, "sendAllLegJointTargets setMotorPositionRefOnly leg=%d joint=%d id=%u", leg, j,
                         (unsigned)id);
                return false;
            }
#endif
        }
    }
    return true;
}

bool DogControl::resendLastJointTargetsWithUpdatedGains() {
    if (!deep_motor_) {
        return false;
    }
    if (!has_last_joint_targets_) {
        return false;
    }
    return sendAllLegJointTargets(last_joint_targets_, last_max_speed_rad_s_);
}

bool DogControl::sendHoldCurrentPoseZeroSpeed(const char* reason, const float preferred_pos[4][LEG_JOINT_COUNT]) {
    if (!deep_motor_) {
        return false;
    }
    float hold_pos[4][LEG_JOINT_COUNT] = {};
    for (int leg = 0; leg < 4; leg++) {
        if (mit_fl_only_ && leg != 0) {
            continue;
        }
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            const uint8_t id = legs_[leg].getMotorId(j);
            if (id == 0) {
                continue;
            }
            if (preferred_pos) {
                hold_pos[leg][j] = preferred_pos[leg][j];
                continue;
            }

            float rad = 0.0f;
            if (!deep_motor_->getMotorActualPosition(id, &rad)) {
                // 读不到反馈时退化为“最近一次已下发目标”；再退化到站立目标，避免发送空姿态。
                if (has_last_joint_targets_) {
                    rad = last_joint_targets_[leg][j];
                } else {
                    rad = legs_[leg].getStanceTargetJoint(j);
                }
            }
            hold_pos[leg][j] = rad;
        }
    }
    clampLegsMechanical(hold_pos);
    const bool ok = sendAllLegJointTargets(hold_pos, DEEP_DOG_MIT_VDES_RAD_S);
    if (ok) {
        ESP_LOGI(TAG, "已补发零速保持帧（%s）", reason ? reason : "unknown");
    }
    return ok;
}

bool DogControl::init() {
    if (initialized_) {
        ESP_LOGI(TAG, "dog init skipped: already initialized");
        return true;
    }
    if (!deep_motor_) {
        ESP_LOGE(TAG, "DeepMotor not set");
        return false;
    }
#if DEEP_DOG_USE_MIT_WALK
    ESP_LOGI(TAG, "整机 init：关节下发为运控帧 (kp=%.2f kd=%.2f tau_ff=%.2f)",
             (double)mit_kp_, (double)mit_kd_, (double)mit_tau_ff_);
#else
    ESP_LOGI(TAG, "整机 init：关节下发为位置参考 PARAM_LOC_REF");
#endif
#if DEEP_DOG_MIT_VALIDATE_FL_ONLY
    ESP_LOGW(TAG, "MIT 台架模式：仅初始化/驱动左前腿(FL)");
#endif
#if DEEP_DOG_USE_MIT_WALK
    // 初始化阶段机器狗趴下：使用更“软”的增益，降低误动作对结构的冲击
    // 经验策略：kp 降低、kd 适当增大（但仍受 setMitGains clamp 约束）
    const float saved_kp = mit_kp_;
    const float saved_kd = mit_kd_;
    const float init_kp = saved_kp * 0.1f;  // kp/10
    const float init_kd = saved_kd * 2.0f;   // kd*2
    setMitGains(init_kp, init_kd);
    ESP_LOGI(TAG, "init 阶段 MIT 增益缩放：kp=%.2f->%.2f kd=%.2f->%.2f（完成后恢复）",
             (double)saved_kp, (double)mit_kp_, (double)saved_kd, (double)mit_kd_);
#endif
    for (int i = 0; i < 4; i++) {
        if (mit_fl_only_ && i != 0) {
            continue;
        }
        if (!legs_[i].init()) {
            ESP_LOGE(TAG, "leg %d init failed", i);
#if DEEP_DOG_USE_MIT_WALK
            setMitGains(saved_kp, saved_kd);
#endif
            return false;
        }
    }
    ESP_LOGI(TAG, "dog init ok%s", mit_fl_only_ ? " (FL only)" : ", 4 legs");
    gait_planner_.resetCycle();
    for (int i = 0; i < 4; i++) {
        legs_[i].setCurrentStep(gait_planner_.effectiveStepForLeg(i));
    }
    initialized_ = true;
    state_machine_.onInitSuccess();
    // 单电机 init 已做“必须收到反馈+零位附近”校验；这里保留一次短延时后的整机兜底检查，便于日志汇总。
    vTaskDelay(pdMS_TO_TICKS(40));
    const bool init_feedback_ok = logMotorActualPositionsAfterInit();
    if (!init_feedback_ok) {
        ESP_LOGE(TAG, "init 后反馈异常，立即整机失能保护");
        (void)disable();
#if DEEP_DOG_USE_MIT_WALK
        setMitGains(saved_kp, saved_kd);
#endif
        return false;
    }
#if DEEP_DOG_USE_MIT_WALK
    setMitGains(saved_kp, saved_kd);
    ESP_LOGI(TAG, "init 完成，已恢复 MIT 增益：kp=%.2f kd=%.2f", (double)mit_kp_, (double)mit_kd_);
#endif
    return true;
}

bool DogControl::logMotorActualPositionsAfterInit() {
    if (!deep_motor_) {
        return false;
    }
    static const char* leg_names[] = {"FL", "FR", "RL", "RR"};
    static const char* joint_names[] = {"HipAA", "HipFE", "Knee"};
    constexpr float kZeroTolRad = 0.15f;  // ~8.6°，趴姿写零后应落在此内
    ESP_LOGI(TAG, "[init 后反馈] 各电机实际角（宜在零位附近），容差 ±%.3f rad", kZeroTolRad);
    int abnormal = 0;
    for (int leg = 0; leg < 4; leg++) {
        if (mit_fl_only_ && leg != 0) {
            continue;
        }
        for (int j = 0; j < LEG_JOINT_COUNT; j++) {
            const uint8_t id = legs_[leg].getMotorId(j);
            if (id == 0) {
                continue;
            }
            float rad = 0.f;
            if (!deep_motor_->getMotorActualPosition(id, &rad)) {
                ESP_LOGW(TAG, "  %s.%s id=%u: 无法读取位置（未注册或无反馈）", leg_names[leg], joint_names[j],
                         (unsigned)id);
                abnormal++;
                continue;
            }
            const float deg = dog_rad_to_deg(rad);
            if (std::fabs(rad) > kZeroTolRad) {
                ESP_LOGW(TAG, "  %s.%s id=%u: pos=%.4f rad (%.1f°) **偏离零位**", leg_names[leg], joint_names[j],
                         (unsigned)id, rad, deg);
                abnormal++;
            } else {
                ESP_LOGI(TAG, "  %s.%s id=%u: pos=%.4f rad (%.1f°) ok", leg_names[leg], joint_names[j],
                         (unsigned)id, rad, deg);
            }
        }
    }
    if (abnormal > 0) {
        ESP_LOGW(TAG, "[init 后反馈] 共 %d 项异常或不可读，请检查趴姿机械零位、CAN 与驱动反馈", abnormal);
        return false;
    } else {
        ESP_LOGI(TAG, "[init 后反馈] 可读关节均在 ±%.3f rad 内，置零外观正常", kZeroTolRad);
        return true;
    }
}

bool DogControl::stand(float max_speed_rad_s) {
    if (!deep_motor_) return false;
#if DEEP_DOG_USE_MIT_WALK
    // MIT：v_des 固定为 DEEP_DOG_MIT_VDES_RAD_S；插值快慢与 max_speed_rad_s 无耦合。
    const float interp_speed = DEEP_DOG_MIT_VDES_RAD_S;
#else
    const float interp_speed = max_speed_rad_s;
#endif

    float target[4][LEG_JOINT_COUNT] = {};
    FillDogStaticPose(DogStaticPoseId::Stand, legs_, target);
    if (!moveToPoseJointsInterp("stand", target, interp_speed)) {
        return false;
    }
    state_machine_.onStandSuccess();
    return true;
}

bool DogControl::lieDown(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    float target[4][LEG_JOINT_COUNT] = {};
    FillDogStaticPose(DogStaticPoseId::LieDownZero, legs_, target);
    if (!moveToPoseJointsInterp("lie_down", target, max_speed_rad_s)) {
        return false;
    }
    state_machine_.onLieDownSuccess();
    return true;
}

bool DogControl::goForwardStepNoEnsure(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    gait_planner_.advanceCycleForward();
    float pos[4][LEG_JOINT_COUNT];
    for (int i = 0; i < 4; i++) {
        const uint16_t step_i = gait_planner_.effectiveStepForLeg(i);
        legs_[i].setCurrentStep(step_i);
        legs_[i].fillStepPositionsAtStepIndex(step_i, pos[i], true);
    }
    clampLegsMechanical(pos);
    logJointTargetsBeforeSend("go_forward", pos);

    return sendAllLegJointTargets(pos, max_speed_rad_s);
}

bool DogControl::goForward(float max_speed_rad_s) {
    if (IsContinuousWalkingPose(state_machine_.state())) {
        ESP_LOGW(TAG, "单步前进被拒绝：当前为持续前进/后退，请先停止");
        return false;
    }
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    state_machine_.beginMove();
    bool ok = goForwardStepNoEnsure(max_speed_rad_s);
    state_machine_.endMove(ok);
    return ok;
}

bool DogControl::goBackStepNoEnsure(float max_speed_rad_s) {
    if (!deep_motor_) return false;
    gait_planner_.advanceCycleBackward();
    float pos[4][LEG_JOINT_COUNT];
    for (int i = 0; i < 4; i++) {
        const uint16_t step_i = gait_planner_.effectiveStepForLeg(i);
        legs_[i].setCurrentStep(step_i);
        legs_[i].fillStepPositionsAtStepIndex(step_i, pos[i], false);
    }
    clampLegsMechanical(pos);
    logJointTargetsBeforeSend("go_back", pos);

    return sendAllLegJointTargets(pos, max_speed_rad_s);
}

bool DogControl::goBack(float max_speed_rad_s) {
    if (IsContinuousWalkingPose(state_machine_.state())) {
        ESP_LOGW(TAG, "单步后退被拒绝：当前为持续前进/后退，请先停止");
        return false;
    }
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    state_machine_.beginMove();
    bool ok = goBackStepNoEnsure(max_speed_rad_s);
    state_machine_.endMove(ok);
    return ok;
}

bool DogControl::goForwardSteps(int steps, float max_speed_rad_s) {
    if (!deep_motor_) {
        return false;
    }
    if (IsContinuousWalkingPose(state_machine_.state())) {
        ESP_LOGW(TAG, "goForwardSteps 被拒绝：当前为持续前进/后退，请先停止");
        return false;
    }
    if (steps <= 0) {
        return true;
    }
    if (steps > 200) {
        ESP_LOGW(TAG, "goForwardSteps: steps=%d 超过 200，已截断", steps);
        steps = 200;
    }
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    state_machine_.beginMove();
    for (int i = 0; i < steps; i++) {
        if (!goForwardStepNoEnsure(max_speed_rad_s)) {
            ESP_LOGE(TAG, "goForwardSteps: 第 %d/%d 步失败", i + 1, steps);
            state_machine_.endMove(false);
            return false;
        }
    }
    state_machine_.endMove(true);
    ESP_LOGI(TAG, "goForwardSteps: 完成 %d 步", steps);
    return true;
}

bool DogControl::goBackSteps(int steps, float max_speed_rad_s) {
    if (!deep_motor_) {
        return false;
    }
    if (IsContinuousWalkingPose(state_machine_.state())) {
        ESP_LOGW(TAG, "goBackSteps 被拒绝：当前为持续前进/后退，请先停止");
        return false;
    }
    if (steps <= 0) {
        return true;
    }
    if (steps > 200) {
        ESP_LOGW(TAG, "goBackSteps: steps=%d 超过 200，已截断", steps);
        steps = 200;
    }
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    state_machine_.beginMove();
    for (int i = 0; i < steps; i++) {
        if (!goBackStepNoEnsure(max_speed_rad_s)) {
            ESP_LOGE(TAG, "goBackSteps: 第 %d/%d 步失败", i + 1, steps);
            state_machine_.endMove(false);
            return false;
        }
    }
    state_machine_.endMove(true);
    ESP_LOGI(TAG, "goBackSteps: 完成 %d 步", steps);
    return true;
}

bool DogControl::goForwardBigStep(float max_speed_rad_s, int inter_step_delay_ms) {
    if (!deep_motor_) {
        return false;
    }
    if (IsContinuousWalkingPose(state_machine_.state())) {
        ESP_LOGW(TAG, "前进一大步被拒绝：当前为持续前进/后退，请先停止");
        return false;
    }
    const uint16_t total = gait_planner_.getTotalSteps();
    if (total < 4) {
        return false;
    }
    const int n = total / 2;
    if (n <= 0) {
        return false;
    }
    inter_step_delay_ms = ClampInt(inter_step_delay_ms, 0, 300);
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    state_machine_.beginMove();
    for (int i = 0; i < n; i++) {
        if (!goForwardStepNoEnsure(max_speed_rad_s)) {
            ESP_LOGE(TAG, "goForwardBigStep: 第 %d/%d 小步失败", i + 1, n);
            state_machine_.endMove(false);
            return false;
        }
        if (i + 1 < n && inter_step_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(inter_step_delay_ms));
        }
    }
    state_machine_.endMove(true);
    ESP_LOGI(TAG, "goForwardBigStep: 完成一大步（%d 小步，半周期）", n);
    return true;
}

bool DogControl::goBackBigStep(float max_speed_rad_s, int inter_step_delay_ms) {
    if (!deep_motor_) {
        return false;
    }
    if (IsContinuousWalkingPose(state_machine_.state())) {
        ESP_LOGW(TAG, "后退一大步被拒绝：当前为持续前进/后退，请先停止");
        return false;
    }
    const uint16_t total = gait_planner_.getTotalSteps();
    if (total < 4) {
        return false;
    }
    const int n = total / 2;
    if (n <= 0) {
        return false;
    }
    inter_step_delay_ms = ClampInt(inter_step_delay_ms, 0, 300);
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    state_machine_.beginMove();
    for (int i = 0; i < n; i++) {
        if (!goBackStepNoEnsure(max_speed_rad_s)) {
            ESP_LOGE(TAG, "goBackBigStep: 第 %d/%d 小步失败", i + 1, n);
            state_machine_.endMove(false);
            return false;
        }
        if (i + 1 < n && inter_step_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(inter_step_delay_ms));
        }
    }
    state_machine_.endMove(true);
    ESP_LOGI(TAG, "goBackBigStep: 完成一大步（%d 小步，半周期）", n);
    return true;
}

void DogControl::setContinuousSpeed(float max_speed_rad_s) {
    const float lo = DEEP_DOG_CHASSIS_SPEED_X100_MIN / 100.0f;
    const float hi = DEEP_DOG_CHASSIS_SPEED_X100_MAX / 100.0f;
    if (max_speed_rad_s < lo) {
        max_speed_rad_s = lo;
    }
    if (max_speed_rad_s > hi) {
        max_speed_rad_s = hi;
    }
    continuous_speed_rad_s_ = max_speed_rad_s;
    ESP_LOGI(TAG, "持续行走电机限速设为 %.2f rad/s", max_speed_rad_s);
}

void DogControl::setMitGains(float kp, float kd) {
    const float kp_new = ClampFloat(kp, 0.0f, 500.0f);
    const float kd_new = ClampFloat(kd, 0.0f, 5.0f);
    mit_kp_ = kp_new;
    mit_kd_ = kd_new;
    ESP_LOGI(TAG, "MIT 增益已更新：kp=%.2f kd=%.2f（后续运控帧生效）", mit_kp_, mit_kd_);
}

void DogControl::setContinuousStepPeriodMs(int ms) {
    continuous_step_period_ms_ = ClampInt(ms, DEEP_DOG_STEP_PERIOD_MS_MIN, DEEP_DOG_STEP_PERIOD_MS_MAX);
    ESP_LOGI(TAG, "持续行走步频周期设为 %d ms", continuous_step_period_ms_);
}

void DogControl::setContinuousCyclePeriodMs(int cycle_period_ms) {
    cycle_period_ms = ClampInt(cycle_period_ms, DEEP_DOG_CYCLE_PERIOD_MS_MIN, DEEP_DOG_CYCLE_PERIOD_MS_MAX);
    const uint16_t steps = gait_planner_.getTotalSteps() > 0 ? gait_planner_.getTotalSteps() : LEG_DEFAULT_TOTAL_STEPS;
    const int step_period_ms = (int)(cycle_period_ms / (int)steps);
    setContinuousStepPeriodMs(step_period_ms > 0 ? step_period_ms : 1);
}

int DogControl::getContinuousCyclePeriodMs() const {
    const uint16_t steps = gait_planner_.getTotalSteps() > 0 ? gait_planner_.getTotalSteps() : LEG_DEFAULT_TOTAL_STEPS;
    return continuous_step_period_ms_ * (int)steps;
}

std::string DogControl::getChassisStatusString() const {
    const DogPoseState s = state_machine_.state();
    const char* pose = "未知";
    switch (s) {
        case DogPoseState::Uninitialized:
            pose = "未初始化";
            break;
        case DogPoseState::UnknownPose:
            pose = "姿态未知";
            break;
        case DogPoseState::Lying:
            pose = "趴下待机";
            break;
        case DogPoseState::Standing:
            pose = "站立";
            break;
        case DogPoseState::Moving:
            pose = "单段行走中";
            break;
        case DogPoseState::WalkingForward:
            pose = "持续前进中";
            break;
        case DogPoseState::WalkingBackward:
            pose = "持续后退中";
            break;
        default:
            break;
    }
    std::string out = std::string("机器狗状态：") + pose;
    if (IsContinuousWalkingPose(s)) {
        char extra[128];
        snprintf(extra, sizeof(extra), "；小步间隔 %d ms（共 %u 步/正弦周期）", continuous_step_period_ms_,
                 (unsigned)gait_planner_.getTotalSteps());
        out += extra;
    } else {
        char extra[96];
        snprintf(extra, sizeof(extra), "；步态周期采样数=%u", (unsigned)gait_planner_.getTotalSteps());
        out += extra;
    }
    return out;
}

void DogControl::ensureContinuousWalkTask() {
    if (continuous_task_handle_) {
        return;
    }
    const BaseType_t ok =
        xTaskCreate(ContinuousWalkTask, "dog_cont_walk", 4096, this, 5, &continuous_task_handle_);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建连续行走任务失败");
        continuous_task_handle_ = nullptr;
    }
}

void DogControl::ContinuousWalkTask(void* arg) {
    DogControl* self = static_cast<DogControl*>(arg);
    if (!self) {
        vTaskDelete(nullptr);
        return;
    }
    const TickType_t idle_delay = pdMS_TO_TICKS(30);
    for (;;) {
        const uint8_t mode = self->continuous_mode_.load(std::memory_order_relaxed);
        const int period_ms =
            self->continuous_step_period_ms_ > 0 ? self->continuous_step_period_ms_ : DEEP_DOG_STEP_PERIOD_MS_DEFAULT;
        const TickType_t step_delay = pdMS_TO_TICKS(period_ms);
        if (mode == 1) {
            if (!self->goForwardStepNoEnsure(self->continuous_speed_rad_s_)) {
                self->stopContinuousLocomotionInternal(false);
            }
            vTaskDelay(step_delay);
        } else if (mode == 2) {
            if (!self->goBackStepNoEnsure(self->continuous_speed_rad_s_)) {
                self->stopContinuousLocomotionInternal(false);
            }
            vTaskDelay(step_delay);
        } else {
            vTaskDelay(idle_delay);
        }
    }
}

void DogControl::stopContinuousLocomotionInternal(bool success) {
    continuous_mode_.store(0, std::memory_order_relaxed);
    state_machine_.endContinuousLocomotion(success);
}

void DogControl::stopContinuousLocomotionIfNeeded() {
    if (IsContinuousWalkingPose(state_machine_.state()) || continuous_mode_.load(std::memory_order_relaxed) != 0) {
        stopContinuousLocomotionInternal(true);
    }
}

void DogControl::stopContinuousLocomotion() {
    stopContinuousLocomotionIfNeeded();
    (void)sendHoldCurrentPoseZeroSpeed("stop_continuous");
}

bool DogControl::isContinuousLocomotionActive() const {
    return IsContinuousWalkingPose(state_machine_.state());
}

bool DogControl::startContinuousForward(float max_speed_rad_s, int step_period_ms) {
    if (!deep_motor_) {
        return false;
    }
    setContinuousSpeed(max_speed_rad_s);
    continuous_step_period_ms_ = ClampInt(step_period_ms, DEEP_DOG_STEP_PERIOD_MS_MIN, DEEP_DOG_STEP_PERIOD_MS_MAX);
    ensureContinuousWalkTask();
    if (!continuous_task_handle_) {
        return false;
    }
    DogPoseState s = state_machine_.state();
    if (s == DogPoseState::WalkingForward) {
        ESP_LOGI(TAG, "持续前进参数已更新");
        return true;
    }
    if (s == DogPoseState::WalkingBackward) {
        stopContinuousLocomotionInternal(true);
        s = state_machine_.state();
    }
    if (s != DogPoseState::Standing) {
        ESP_LOGW(TAG, "持续前进被拒绝：需先站立（当前非站立/持续态）");
        return false;
    }
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    if (state_machine_.state() != DogPoseState::Standing) {
        ESP_LOGW(TAG, "持续前进被拒绝：未能进入站立");
        return false;
    }
    state_machine_.beginContinuousForward();
    continuous_mode_.store(1, std::memory_order_relaxed);
    ESP_LOGI(TAG, "持续前进已启动（限速 %.2f rad/s，间隔 %d ms）", continuous_speed_rad_s_, continuous_step_period_ms_);
    return true;
}

bool DogControl::startContinuousBackward(float max_speed_rad_s, int step_period_ms) {
    if (!deep_motor_) {
        return false;
    }
    setContinuousSpeed(max_speed_rad_s);
    continuous_step_period_ms_ = ClampInt(step_period_ms, DEEP_DOG_STEP_PERIOD_MS_MIN, DEEP_DOG_STEP_PERIOD_MS_MAX);
    ensureContinuousWalkTask();
    if (!continuous_task_handle_) {
        return false;
    }
    DogPoseState s = state_machine_.state();
    if (s == DogPoseState::WalkingBackward) {
        ESP_LOGI(TAG, "持续后退参数已更新");
        return true;
    }
    if (s == DogPoseState::WalkingForward) {
        stopContinuousLocomotionInternal(true);
        s = state_machine_.state();
    }
    if (s != DogPoseState::Standing) {
        ESP_LOGW(TAG, "持续后退被拒绝：需先站立（当前非站立/持续态）");
        return false;
    }
    if (!ensureStandingForWalk(max_speed_rad_s)) {
        return false;
    }
    if (state_machine_.state() != DogPoseState::Standing) {
        ESP_LOGW(TAG, "持续后退被拒绝：未能进入站立");
        return false;
    }
    state_machine_.beginContinuousBackward();
    continuous_mode_.store(2, std::memory_order_relaxed);
    ESP_LOGI(TAG, "持续后退已启动（限速 %.2f rad/s，间隔 %d ms）", continuous_speed_rad_s_, continuous_step_period_ms_);
    return true;
}

bool DogControl::disable() {
    stopContinuousLocomotionIfNeeded();
    for (int i = 0; i < 4; i++) {
        if (mit_fl_only_ && i != 0) {
            continue;
        }
        if (!legs_[i].disable()) {
            ESP_LOGE(TAG, "leg %d disable failed", i);
            return false;
        }
    }
    state_machine_.onMotorSystemDisabled();
    initialized_ = false;
    return true;
}

bool DogControl::dance(float max_speed_rad_s) {
    return danceWithMode("default", 0, 3, max_speed_rad_s);
}

static uint32_t DanceNextRand(uint32_t* s) {
    // LCG：足够用于动作随机编排
    *s = (*s) * 1664525u + 1013904223u;
    return *s;
}

bool DogControl::danceWithMode(const std::string& mode, int seed, int rounds, float max_speed_rad_s) {
    rounds = ClampInt(rounds, 1, 12);
    if (!stand(max_speed_rad_s)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(450));

    const int stride_delay_ms = 45;
    const int pose_hold_ms = 180;

    if (mode == "random") {
        uint32_t rng = (seed != 0) ? (uint32_t)seed : esp_random();
        for (int i = 0; i < rounds; i++) {
            const uint32_t r = DanceNextRand(&rng);
            const int pick = (int)(r % 6);
            if (pick == 0) {
                if (!goForwardBigStep(max_speed_rad_s, stride_delay_ms)) return false;
            } else if (pick == 1) {
                if (!goBackBigStep(max_speed_rad_s, stride_delay_ms)) return false;
            } else if (pick == 2) {
                if (!goToStaticPose(DogStaticPoseId::TiltLeft, max_speed_rad_s)) return false;
                vTaskDelay(pdMS_TO_TICKS(pose_hold_ms));
            } else if (pick == 3) {
                if (!goToStaticPose(DogStaticPoseId::TiltRight, max_speed_rad_s)) return false;
                vTaskDelay(pdMS_TO_TICKS(pose_hold_ms));
            } else if (pick == 4) {
                if (!goToStaticPose(DogStaticPoseId::FrontDownBackUp, max_speed_rad_s)) return false;
                vTaskDelay(pdMS_TO_TICKS(pose_hold_ms));
            } else {
                if (!goToStaticPose(DogStaticPoseId::FrontUpBackDown, max_speed_rad_s)) return false;
                vTaskDelay(pdMS_TO_TICKS(pose_hold_ms));
            }
            // 每轮拉回站立，避免随机串联误差累积导致姿态漂移
            if (!stand(max_speed_rad_s)) return false;
            vTaskDelay(pdMS_TO_TICKS(140));
        }
        ESP_LOGI(TAG, "dance done (random rounds=%d seed=%u)", rounds, (unsigned)rng);
        return true;
    }

    // default：按阶段计划里的“站立/前后走 + 左右倾 + 前后俯仰”编排
    // 1) 左右倾 * rounds
    for (int i = 0; i < rounds; i++) {
        if (!goToStaticPose(DogStaticPoseId::TiltLeft, max_speed_rad_s)) return false;
        vTaskDelay(pdMS_TO_TICKS(pose_hold_ms));
        if (!goToStaticPose(DogStaticPoseId::TiltRight, max_speed_rad_s)) return false;
        vTaskDelay(pdMS_TO_TICKS(pose_hold_ms));
    }
    if (!stand(max_speed_rad_s)) return false;
    vTaskDelay(pdMS_TO_TICKS(220));

    // 2) 前后俯仰各一次
    if (!goToStaticPose(DogStaticPoseId::FrontDownBackUp, max_speed_rad_s)) return false;
    vTaskDelay(pdMS_TO_TICKS(pose_hold_ms + 80));
    if (!goToStaticPose(DogStaticPoseId::FrontUpBackDown, max_speed_rad_s)) return false;
    vTaskDelay(pdMS_TO_TICKS(pose_hold_ms + 80));
    if (!stand(max_speed_rad_s)) return false;
    vTaskDelay(pdMS_TO_TICKS(260));

    // 3) 前进/后退大步（展示移动能力）
    for (int i = 0; i < 2; i++) {
        if (!goForwardBigStep(max_speed_rad_s, stride_delay_ms)) return false;
        vTaskDelay(pdMS_TO_TICKS(380));
    }
    for (int i = 0; i < 2; i++) {
        if (!goBackBigStep(max_speed_rad_s, stride_delay_ms)) return false;
        vTaskDelay(pdMS_TO_TICKS(380));
    }
    if (!stand(max_speed_rad_s)) {
        return false;
    }
    ESP_LOGI(TAG, "dance done (default: sway x%d + pitch + big-step)", rounds);
    return true;
}

// --- MCP 工具注册 ---

void RegisterDogMcpTools(McpServer& mcp_server, DogControl* dog) {
    if (!dog) return;

    mcp_server.AddTool("self.dog.init", "机器狗整机初始化（使能 12 个电机）。用户说：初始化、整机初始化、机器狗初始化 时调用", PropertyList(), [dog](const PropertyList&) -> ReturnValue {
        if (dog->init()) return std::string("机器狗初始化成功");
        return std::string("机器狗初始化失败");
    });

    mcp_server.AddTool("self.dog.stand", "机器狗整机站立，四条腿回站立位。用户说：站起来、站立、站起、机器狗站立 时调用", PropertyList(), [dog](const PropertyList&) -> ReturnValue {
        if (dog->stand()) return std::string("机器狗已站立");
        return std::string("机器狗站立失败");
    });

    mcp_server.AddTool("self.dog.lie_down", "机器狗整机卧倒，四条腿回零位。用户说：卧倒、趴下、回零位、机器狗卧倒 时调用", PropertyList(), [dog](const PropertyList&) -> ReturnValue {
        if (dog->lieDown()) return std::string("机器狗已卧倒");
        return std::string("机器狗卧倒失败");
    });

    mcp_server.AddTool("self.dog.disable", "机器狗整机失能（12 电机 reset/停扭）。用户说：整体断电、整体失能、电机下电 时调用",
                        PropertyList(), [dog](const PropertyList&) -> ReturnValue {
        if (dog->disable()) return std::string("机器狗已整体失能（电机停扭）");
        return std::string("机器狗整体失能失败");
    });

    mcp_server.AddTool("self.chassis.forward_big",
                         "机器狗向前「一大步」：沿相位连续迈半个正弦周期（若干小步+可选延时），非持续行走。参数 step_delay_ms：小步之间延时 0~300ms",
                         PropertyList(std::vector<Property>{Property("step_delay_ms", kPropertyTypeInteger, 40, 0, 300)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const int d = props["step_delay_ms"].value<int>();
                             if (dog->goForwardBigStep(DEEP_DOG_MIT_VDES_RAD_S, d)) {
                                 return std::string("已向前一大步");
                             }
                             return std::string("前进一大步失败");
                         });

    mcp_server.AddTool("self.chassis.backward_big",
                         "机器狗向后「一大步」：半个正弦周期的小步串联。参数 step_delay_ms 同 forward_big",
                         PropertyList(std::vector<Property>{Property("step_delay_ms", kPropertyTypeInteger, 40, 0, 300)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const int d = props["step_delay_ms"].value<int>();
                             if (dog->goBackBigStep(DEEP_DOG_MIT_VDES_RAD_S, d)) {
                                 return std::string("已向后一大步");
                             }
                             return std::string("后退一大步失败");
                         });

    mcp_server.AddTool("self.chassis.start_forward",
                         "机器狗持续向前行走，直到 stop。参数 step_period_ms：小步间隔越小步频越快",
                         PropertyList(std::vector<Property>{Property("step_period_ms", kPropertyTypeInteger,
                                                                     DEEP_DOG_STEP_PERIOD_MS_DEFAULT,
                                                                     DEEP_DOG_STEP_PERIOD_MS_MIN,
                                                                     DEEP_DOG_STEP_PERIOD_MS_MAX)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const int p = props["step_period_ms"].value<int>();
                             if (dog->startContinuousForward(dog->getContinuousSpeed(), p)) {
                                 return std::string("已开始持续前进");
                             }
                             return std::string("持续前进启动失败");
                         });

    mcp_server.AddTool("self.chassis.start_backward",
                         "机器狗持续向后退，直到 stop。参数 step_period_ms 同 start_forward",
                         PropertyList(std::vector<Property>{Property("step_period_ms", kPropertyTypeInteger,
                                                                     DEEP_DOG_STEP_PERIOD_MS_DEFAULT,
                                                                     DEEP_DOG_STEP_PERIOD_MS_MIN,
                                                                     DEEP_DOG_STEP_PERIOD_MS_MAX)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const int p = props["step_period_ms"].value<int>();
                             if (dog->startContinuousBackward(dog->getContinuousSpeed(), p)) {
                                 return std::string("已开始持续后退");
                             }
                             return std::string("持续后退启动失败");
                         });

    mcp_server.AddTool("self.chassis.stop",
                         "停止持续前进/后退，关节保持当前姿态。用户说：停下、停止、别走了 时调用",
                         PropertyList(),
                         [dog](const PropertyList&) -> ReturnValue {
                             dog->stopContinuousLocomotion();
                             return std::string("已停止持续行走");
                         });

    mcp_server.AddTool("self.chassis.status",
                         "查询机器狗当前姿态与行走状态（中文）",
                         PropertyList(),
                         [dog](const PropertyList&) -> ReturnValue {
                             return dog->getChassisStatusString();
                         });

    mcp_server.AddTool("self.chassis.set_gait_steps",
                         "设置一个正弦周期采样点数（越大步态越细腻，范围 8~120）",
                         PropertyList(std::vector<Property>{Property("steps", kPropertyTypeInteger, (int)LEG_DEFAULT_TOTAL_STEPS, 8, 120)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const int steps = props["steps"].value<int>();
                             dog->setGaitTotalSteps((uint16_t)steps);
                             return std::string("步态周期采样点数已设置为 ") + std::to_string(steps);
                         });

    mcp_server.AddTool("self.chassis.set_speed",
                         "设置完整步态周期（ms），采样点数不变时：周期越小跑得越快。示例：cycle_period_ms=2000 表示步态周期 2s",
                         PropertyList(std::vector<Property>{
                             Property("cycle_period_ms", kPropertyTypeInteger, DEEP_DOG_CYCLE_PERIOD_MS_DEFAULT,
                                      DEEP_DOG_CYCLE_PERIOD_MS_MIN, DEEP_DOG_CYCLE_PERIOD_MS_MAX)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const int cycle_ms = props["cycle_period_ms"].value<int>();
                             dog->setContinuousCyclePeriodMs(cycle_ms);
                             return std::string("已更新：cycle_period_ms=") +
                                    std::to_string(dog->getContinuousCyclePeriodMs()) +
                                    "ms, step_period_ms=" + std::to_string(dog->getContinuousStepPeriodMs()) +
                                    "ms";
                         });

    mcp_server.AddTool("self.chassis.set_mit_gains",
                         "设置 MIT 全局增益（后续所有关节运控帧默认使用）。kp 范围 0~500；kd 范围 0~5",
                         PropertyList(std::vector<Property>{
                             Property("kp_x10", kPropertyTypeInteger, 100, 0, 5000),
                             Property("kd_x100", kPropertyTypeInteger, 150, 0, 500)}),
                         [dog](const PropertyList& props) -> ReturnValue {
                             const float kp = props["kp_x10"].value<int>() / 10.0f;
                             const float kd = props["kd_x100"].value<int>() / 100.0f;
                             dog->setMitGains(kp, kd);
                             const bool resent = dog->resendLastJointTargetsWithUpdatedGains();
                             return std::string("MIT 增益已设置：kp=") + std::to_string(dog->getMitKp()) +
                                    ", kd=" + std::to_string(dog->getMitKd()) +
                                    (resent ? "（已用新增益重发上一帧）" : "（尚无可重发的上一帧）");
                         });

    mcp_server.AddTool("self.chassis.get_mit_gains",
                         "查询当前 MIT 全局增益（kp/kd）",
                         PropertyList(),
                         [dog](const PropertyList&) -> ReturnValue {
                             return std::string("当前 MIT 增益：kp=") + std::to_string(dog->getMitKp()) +
                                    ", kd=" + std::to_string(dog->getMitKd());
                         });

    mcp_server.AddTool("self.chassis.pose",
                       "切换到静态姿态点（插值到目标点）。pose: lie_down_zero/stand/tilt_left/tilt_right/front_down_back_up/front_up_back_down",
                       PropertyList(std::vector<Property>{
                           Property("pose", kPropertyTypeString, std::string("stand")),
                       }),
                       [dog](const PropertyList& props) -> ReturnValue {
                           const std::string pose = props["pose"].value<std::string>();
                           DogStaticPoseId id = DogStaticPoseId::Stand;
                           if (pose == "lie_down_zero" || pose == "lie_down" || pose == "zero") {
                               id = DogStaticPoseId::LieDownZero;
                           } else if (pose == "stand") {
                               id = DogStaticPoseId::Stand;
                           } else if (pose == "tilt_left") {
                               id = DogStaticPoseId::TiltLeft;
                           } else if (pose == "tilt_right") {
                               id = DogStaticPoseId::TiltRight;
                           } else if (pose == "front_down_back_up") {
                               id = DogStaticPoseId::FrontDownBackUp;
                           } else if (pose == "front_up_back_down") {
                               id = DogStaticPoseId::FrontUpBackDown;
                           } else {
                               return std::string("未知 pose：") + pose;
                           }
                           if (dog->goToStaticPose(id)) {
                               return std::string("已切换姿态：") + pose;
                           }
                           return std::string("切换姿态失败：") + pose;
                       });

    mcp_server.AddTool("self.chassis.dance",
                       "机器狗跳舞（支持可编排/可随机）。mode=default/random，rounds=左右倾重复次数，seed=随机种子(0表示自动)",
                       PropertyList(std::vector<Property>{
                           Property("mode", kPropertyTypeString, std::string("default")),
                           Property("rounds", kPropertyTypeInteger, 3, 1, 12),
                           Property("seed", kPropertyTypeInteger, 0, 0, 2147483647),
                       }),
                       [dog](const PropertyList& props) -> ReturnValue {
                           const std::string mode = props["mode"].value<std::string>();
                           const int rounds = props["rounds"].value<int>();
                           const int seed = props["seed"].value<int>();
                           if (dog->danceWithMode(mode, seed, rounds)) return std::string("跳舞完成");
                           return std::string("跳舞失败");
                       });

    ESP_LOGI(TAG,
             "Dog MCP: dog.init/stand/lie_down/disable; chassis.forward_big/backward_big/start_forward/start_backward/stop/status/"
             "set_speed/set_gait_steps/set_mit_gains/get_mit_gains/pose/dance");
}
