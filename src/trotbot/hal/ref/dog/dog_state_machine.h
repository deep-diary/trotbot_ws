#ifndef DEEP_DOG_STATE_MACHINE_H
#define DEEP_DOG_STATE_MACHINE_H

#include <stdint.h>

/**
 * 整机姿态/任务状态（供 DogControl 与板级 LED 等扩展使用）。
 *
 * - 趴下 (Lying) 视为待机：前进/后退会先自动站立，再迈步。
 * - 未初始化时禁止行走。
 */
enum class DogPoseState : uint8_t {
    Uninitialized = 0, ///< 未 init 或已 disable
    /** 行走指令失败等，姿态未确认（需重新站立或卧倒）；init 成功默认不用此状态 */
    UnknownPose,
    Lying,    ///< 卧倒、待机（**整机初始化在趴姿下完成并写零位后，init 成功即为此状态**）
    Standing, ///< 站立，可安全行走
    Moving,   ///< 正在执行行走指令（单步或连续步序列）
    /** 长按触摸键触发的持续前进/后退（由 DogControl 后台任务迈步，直至停止） */
    WalkingForward,
    WalkingBackward,
};

/**
 * 轻量状态机：仅维护枚举与合法迁移，不做 CAN 操作。
 */
class DogStateMachine {
public:
    DogPoseState state() const { return state_; }

    void onMotorSystemDisabled() { state_ = DogPoseState::Uninitialized; }

    /** 初始化在趴姿下进行、电机关零后姿态已知，视为趴下 */
    void onInitSuccess() {
        if (state_ == DogPoseState::Uninitialized) {
            state_ = DogPoseState::Lying;
        }
    }

    void onStandSuccess() { state_ = DogPoseState::Standing; }

    void onLieDownSuccess() { state_ = DogPoseState::Lying; }

    /** 行走序列开始（单步或连续步外层各调用一次） */
    void beginMove() {
        if (state_ == DogPoseState::Standing) {
            state_ = DogPoseState::Moving;
        }
    }

    /** 行走序列结束；失败时保守标为 UnknownPose（姿态不确定） */
    void endMove(bool success) {
        if (state_ == DogPoseState::Moving) {
            state_ = success ? DogPoseState::Standing : DogPoseState::UnknownPose;
        }
    }

    /** 持续前进开始（仅自 Standing） */
    void beginContinuousForward() {
        if (state_ == DogPoseState::Standing) {
            state_ = DogPoseState::WalkingForward;
        }
    }

    /** 持续后退开始（仅自 Standing） */
    void beginContinuousBackward() {
        if (state_ == DogPoseState::Standing) {
            state_ = DogPoseState::WalkingBackward;
        }
    }

    /** 持续行走结束：成功则回到站立（关节保持最后下发姿态）；失败为 UnknownPose */
    void endContinuousLocomotion(bool success) {
        if (state_ == DogPoseState::WalkingForward || state_ == DogPoseState::WalkingBackward) {
            state_ = success ? DogPoseState::Standing : DogPoseState::UnknownPose;
        }
    }

    /** 是否需要在迈步前先站立（趴下或姿态未知） */
    bool needsStandBeforeWalk() const {
        return state_ == DogPoseState::Lying || state_ == DogPoseState::UnknownPose;
    }

private:
    DogPoseState state_ = DogPoseState::Uninitialized;
};

#endif // DEEP_DOG_STATE_MACHINE_H
