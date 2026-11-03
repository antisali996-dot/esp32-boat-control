#pragma once
#include <cstdint>
#include <cstdio>

#include "motor_control.h"

// 首次故障证据（纯逻辑，可在电脑上用 g++ 回归）。
//
// 规则（与需求逐条对应）：
//   - 每个有效测速窗口，在调用 motor.update 之前先构造“当前记录”，
//     其中 pwm/integral 取故障判断之前的值；
//   - 同时保留上一有效窗口的同类记录；
//   - 只有故障从 NONE 首次变为非 NONE 时才锁存，之后不再覆盖；
//   - 没有上一窗口时 PREVIOUS 全为 0，不虚造测量；
//   - 不提供任何清除入口（只有上电重新构造才归零）。

struct MeasureRecord {
  uint32_t ms = 0;
  float rpm = 0;
  int32_t pwm = 0;
  uint32_t pulses = 0;
  uint32_t elapsed = 0;
  float integral = 0;
};

struct FaultEvidence {
  bool latched = false;
  uint32_t id = 0;                 // 本次启动内从 1 开始
  MotorControl::Fault fault = MotorControl::NONE;
  MeasureRecord current;           // 故障判断之前的本次记录
  MeasureRecord previous;          // 上一有效窗口；没有则保持全 0
  bool hasPrevious = false;

  // 上一次有效窗口记录，供下一窗口引用。
  MeasureRecord lastWindow;
  bool hasLastWindow = false;
  uint32_t counter = 0;

  const char *faultName() const {
    switch (fault) {
      case MotorControl::NO_PULSES: return "NO_ENCODER_PULSES";
      case MotorControl::LOW_SPEED: return "SUSTAINED_LOW_SPEED";
      case MotorControl::OVERSPEED: return "OVERSPEED";
      default: return "NONE";
    }
  }

  // 在 motor.update 之前调用：构造本次记录并备份上一窗口。
  // pwm/integral 必须是故障判断之前的值。
  MeasureRecord beginWindow(uint32_t ms, float rpm, int32_t pwm, uint32_t pulses,
                            uint32_t elapsed, float integral) {
    MeasureRecord cur;
    cur.ms = ms;
    cur.rpm = rpm;
    cur.pwm = pwm;
    cur.pulses = pulses;
    cur.elapsed = elapsed;
    cur.integral = integral;
    return cur;
  }

  // motor.update 之后调用：若首次出现故障则锁存。
  void afterUpdate(MotorControl::Fault before, MotorControl::Fault after,
                   const MeasureRecord &cur) {
    // 更新“上一有效窗口”指针（无论是否故障，本窗口都已有效）。
    // 锁存用的是故障发生前的上一窗口，所以在判定之前先记录。
    if (!latched && before == MotorControl::NONE && after != MotorControl::NONE) {
      latched = true;
      id = ++counter;
      fault = after;
      current = cur;
      if (hasLastWindow) {
        previous = lastWindow;
        hasPrevious = true;
      } else {
        previous = MeasureRecord{};   // 全 0，不虚造
        hasPrevious = false;
      }
    }
    // 本窗口成为下一窗口的“上一有效窗口”。
    lastWindow = cur;
    hasLastWindow = true;
  }

  // 格式化：返回是否成功（不发送半行由调用者判断）。
  // PREVIOUS 行与 CURRENT 行使用同一 id。
  static bool formatCurrent(char *out, size_t cap, const FaultEvidence &e) {
    const int n = snprintf(out, cap,
        "JAY FAULT CURRENT id=%lu name=%s ms=%lu rpm=%.2f pwm=%ld pulses=%lu elapsed=%lu integral=%.2f",
        (unsigned long)e.id, e.faultName(), (unsigned long)e.current.ms,
        (double)e.current.rpm, (long)e.current.pwm, (unsigned long)e.current.pulses,
        (unsigned long)e.current.elapsed, (double)e.current.integral);
    return n > 0 && (size_t)n < cap;
  }

  static bool formatPrevious(char *out, size_t cap, const FaultEvidence &e) {
    const int n = snprintf(out, cap,
        "JAY FAULT PREVIOUS id=%lu ms=%lu rpm=%.2f pwm=%ld pulses=%lu elapsed=%lu integral=%.2f",
        (unsigned long)e.id, (unsigned long)e.previous.ms, (double)e.previous.rpm,
        (long)e.previous.pwm, (unsigned long)e.previous.pulses,
        (unsigned long)e.previous.elapsed, (double)e.previous.integral);
    return n > 0 && (size_t)n < cap;
  }
};
