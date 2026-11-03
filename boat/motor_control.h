#pragma once
#include <stdint.h>

// 纯逻辑控制器：不依赖 Arduino，便于在电脑上用 g++ 直接回归测试。
// 故障一旦锁存只有重新上电才清除，本结构不提供任何自动清故障路径。
struct MotorControl {
  enum Fault { NONE, NO_PULSES, LOW_SPEED, OVERSPEED };

  int pwm = 0;
  float integral = 0;
  uint32_t lastMotion = 0, slowSince = 0;
  bool trackingSlow = false;
  Fault fault = NONE;

  // 超速触发证据快照：锁存后保留，供串口一次性报告。
  float overspeedRpm = 0;
  int overspeedPwm = 0;
  uint32_t overspeedPulses = 0, overspeedIntervalMs = 0;

  // 最近一个控制周期实际采到的脉冲数，供运行日志使用（不参与控制）。
  uint32_t lastPulses = 0;

  static float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  // targetRpm 为目标转速，overspeedRpm 为软件超速阈值。
  void update(uint32_t now, uint32_t elapsed, uint32_t pulses,
              float rpm, float targetRpm, float overspeedRpmRef, bool enabled) {
    lastPulses = pulses;
    // 超速优先判定：达到阈值立刻切断动力并锁存，不等待下一轮、不因转速回落自动恢复。
    if (fault == NONE && enabled && elapsed > 0 && rpm >= overspeedRpmRef) {
      overspeedRpm = rpm;
      overspeedPwm = pwm;
      overspeedPulses = pulses;
      overspeedIntervalMs = elapsed;
      fault = OVERSPEED;
      pwm = 0;
      integral = 0;
      return;
    }

    if (pulses > 0 || pwm < 80) lastMotion = now;
    if (enabled && pwm >= 80 && uint32_t(now - lastMotion) >= 1500) fault = NO_PULSES;
    // 高输出而持续严重掉速：包括仍有脉冲的过载/部分卡滞。
    const bool slow = enabled && pwm >= 220 && rpm < targetRpm * 0.65f;
    if (!slow) trackingSlow = false;
    else if (!trackingSlow) { trackingSlow = true; slowSince = now; }
    else if (uint32_t(now - slowSince) >= 2000) fault = LOW_SPEED;
    if (!enabled || fault != NONE) {
      pwm = 0;
      integral = 0;
      return;
    }
    const float error = targetRpm - rpm;
    const float proportional = 0.18f * error;
    integral = clamp(integral + 0.8f * error * (elapsed / 1000.0f), 0, 255);
    const int desired = (int)clamp(proportional + integral + 0.001f, 0, 255);
    const int applied = desired > pwm + 10 ? pwm + 10 : desired;
    pwm = applied;
    // 抗积分饱和：积分跟随“真正下发的输出”，而不是理想输出。
    // 输出被启动斜率或上限约束时，不再继续积累推动加速的积分。
    if (proportional >= 0) {
      const float room = (float)pwm - proportional;
      integral = room > 0 ? (integral > room ? room : integral) : 0;
    }
  }
};
