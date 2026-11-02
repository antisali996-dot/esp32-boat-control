#include <cassert>
#include <cstdio>
#include "../boat/motor_control.h"
#include "../boat/servo_pulse.h"
#include "../remote/steering.h"
#include "../remote/tx_state.h"

int main() {
  assert(steeringPulse(60) == 1167 && steeringPulse(90) == 1500 && steeringPulse(120) == 1833);
  assert(steeringPulse(0) == 1167 && steeringPulse(180) == 1833);
  MotorControl saturated;
  for (uint32_t now = 50; now <= 3000; now += 50)
    saturated.update(now, 50, 48, 400, 580, true);
  assert(saturated.pwm == 255 && saturated.fault == MotorControl::NONE);
  saturated.update(3050, 50, 100, 850, 580, true);
  assert(saturated.pwm < 255);

  MotorControl slow;
  for (uint32_t now = 50; now <= 10000; now += 50)
    slow.update(now, 50, 12, 100, 580, true);
  assert(slow.pwm == 0 && slow.fault == MotorControl::LOW_SPEED);
  slow.update(10050, 50, 70, 580, 580, true);
  assert(slow.pwm == 0); // 故障锁定，不随脉冲恢复自动重启。

  MotorControl missing;
  for (uint32_t now = 50; now <= 3000; now += 50)
    missing.update(now, 50, 0, 0, 580, true);
  assert(missing.pwm == 0 && missing.fault == MotorControl::NO_PULSES);

  MotorControl normal;
  normal.update(50, 50, 40, 333, 580, true);
  assert(normal.pwm == 10); // 启动斜率限制。
  normal.update(100, 50, 70, 580, 580, false);
  assert(normal.pwm == 0);

  assert(!sendTimedOut(false, 1000, 0, 500)); // 回调已完成，不使用旧 pending。
  assert(sendTimedOut(true, 1000, 0, 500));
  assert(!sendTimedOut(true, 30, UINT32_MAX - 20, 500));
  for (int raw = 0; raw <= 4095; ++raw) {
    int angle = steeringAngle(raw);
    assert(angle >= 60 && angle <= 120);
    if (raw) assert(angle >= steeringAngle(raw - 1) && angle - steeringAngle(raw - 1) <= 1);
    if (raw >= DEAD_LOW && raw <= DEAD_HIGH) assert(angle == 90);
  }
  assert(steeringAngle(0) == 60 && steeringAngle(4095) == 120);
  puts("PASS: saturation, unloading, low-speed fault, no-pulse fault, fault latch, ramp, disabled output, TX timeout, rollover, all ADC inputs");
}
