#pragma once

// 脉宽与身份串同源：identity 里的数值与实际下发脉宽由 static_assert 绑定。
#include "fw_identity.h"

constexpr int SERVO_CENTER_US = FW_CENTER_US;
constexpr int SERVO_SWING_US = FW_SWING_US;
static_assert(SERVO_CENTER_US - SERVO_SWING_US == FW_MIN_US, "Center/swing must match FW_MIN_US");
static_assert(SERVO_CENTER_US + SERVO_SWING_US == FW_MAX_US, "Center/swing must match FW_MAX_US");
static_assert(FW_CENTER_US - FW_SWING_US >= 1000 && FW_CENTER_US + FW_SWING_US <= 2000,
              "Servo pulse exceeds working limits");
static_assert(FW_MIN_ANGLE == FW_CENTER_ANGLE - FW_STEER_DEG &&
              FW_MAX_ANGLE == FW_CENTER_ANGLE + FW_STEER_DEG,
              "Angle range must be center +/- steering");
static_assert(FW_STEER_DEG == 20, "This build ships the 20-degree steering contract");
// 14 位、50Hz 下每 tick 为 20000/16384us，最长脉宽必须落在 14 位计数范围内。
static_assert(FW_MAX_US <= 16383 * 20000 / 16384, "Pulse width exceeds 14-bit tick range");

inline int steeringPulse(int angle) {
  if (angle < FW_MIN_ANGLE) angle = FW_MIN_ANGLE;
  if (angle > FW_MAX_ANGLE) angle = FW_MAX_ANGLE;
  return SERVO_CENTER_US + (angle - FW_CENTER_ANGLE) * SERVO_SWING_US / FW_STEER_DEG;
}

// 只在最终目标脉宽变化时才更新舵机输出，避免无意义的重复下发。
// lastPulseUs 初值 -1，保证首次回中一定会输出一次。
struct ServoOutput {
  int lastPulseUs = -1;
  int writes = 0;
  // 返回 true 表示本次目标脉宽确实发生了变化，调用者才需要下发。
  bool shouldWrite(int pulseUs) {
    if (pulseUs == lastPulseUs) return false;
    lastPulseUs = pulseUs;
    ++writes;
    return true;
  }
};
