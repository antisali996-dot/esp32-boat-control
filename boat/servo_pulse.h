#pragma once

constexpr int SERVO_CENTER_US = 1500;
// 依据标称 500～2500us / 180°，30°对应约333us；物理角度仍需实测。
constexpr int SERVO_SWING_US = 333;
static_assert(SERVO_CENTER_US - SERVO_SWING_US >= 1000 &&
              SERVO_CENTER_US + SERVO_SWING_US <= 2000, "Servo pulse exceeds working limits");
inline int steeringPulse(int angle) {
  if (angle < 60) angle = 60;
  if (angle > 120) angle = 120;
  return SERVO_CENTER_US + (angle - 90) * SERVO_SWING_US / 30;
}
