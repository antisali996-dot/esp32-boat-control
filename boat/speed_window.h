#pragma once
#include <stdint.h>

// 测速窗口（纯逻辑，可在电脑上回归测试）。
//
// 旧实现的问题：首个控制周期在 `lastControl` 还等于 millis() 时就把计数器清零，
// 于是把“上电以来累计的脉冲”除以一个并非真实测量间隔的 elapsed，
// 既可能算出荒谬的高转速，也可能因为 elapsed 很小而直接触发假超速。
//
// 本实现：
//   1. begin() 在同一临界区里清零计数并建立时间基准；首轮不产生转速样本；
//   2. 首轮只建立基准，PWM 保持由控制器决定的零输出（调用者传入 enabled=false）；
//   3. 之后每次都按“真实达到的最小控制周期”换算转速。

struct SpeedWindow {
  uint32_t baselineMs = 0;
  bool primed = false;
  uint32_t minIntervalMs = 50;   // 实际使用的控制周期下限，防止除零/放大噪声

  // 在同一临界区内（由调用者保证）清零计数并记录时间基准。
  void begin(uint32_t now) {
    baselineMs = now;
    primed = true;
  }

  // 距离上次基准已经过去的毫秒数。
  uint32_t elapsed(uint32_t now) const { return now - baselineMs; }

  bool ready(uint32_t now) const { return primed && elapsed(now) >= minIntervalMs; }

  // 用真实区间把脉冲换算成 RPM。intervalMs 必须是实际经过的时间。
  // 未标定的每转脉冲数由调用者传入（PULSES_PER_REV），这里不做任何假设。
  static float rpmFromPulses(uint32_t pulses, uint32_t intervalMs, float pulsesPerRev) {
    if (intervalMs == 0 || pulsesPerRev <= 0) return 0.0f;
    return pulses * 60000.0f / (pulsesPerRev * intervalMs);
  }
};
