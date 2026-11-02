#pragma once
#include <stdint.h>

struct MotorControl {
  enum Fault { NONE, NO_PULSES, LOW_SPEED };
  int pwm = 0;
  float integral = 0;
  uint32_t lastMotion = 0, slowSince = 0;
  bool trackingSlow = false;
  Fault fault = NONE;

  static float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }
  void update(uint32_t now, uint32_t elapsed, uint32_t pulses,
              float rpm, float target, bool enabled) {
    if (pulses > 0 || pwm < 80) lastMotion = now;
    if (enabled && pwm >= 80 && uint32_t(now - lastMotion) >= 1500) fault = NO_PULSES;
    // 高输出而持续严重掉速：包括仍有脉冲的过载/部分卡滞。
    const bool slow = enabled && pwm >= 220 && rpm < target * 0.65f;
    if (!slow) trackingSlow = false;
    else if (!trackingSlow) { trackingSlow = true; slowSince = now; }
    else if (uint32_t(now - slowSince) >= 2000) fault = LOW_SPEED;
    if (!enabled || target <= 0 || fault != NONE) {
      pwm = 0;
      integral = 0;
      return;
    }
    const float error = target - rpm;
    const float proportional = 0.18f * error;
    integral = clamp(integral + 0.8f * error * (elapsed / 1000.0f), 0, 255);
    // 积分允许抵达饱和边界，但不继续积累正向超额。
    if (proportional >= 0) integral = clamp(integral, 0, clamp(255 - proportional, 0, 255));
    const int desired = (int)clamp(proportional + integral + 0.001f, 0, 255);
    pwm = desired > pwm + 10 ? pwm + 10 : desired;
  }
};
