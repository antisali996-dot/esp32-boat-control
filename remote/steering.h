#pragma once

// 保留原程序死区；摇杆实际端点应通过串口读数校准。
constexpr int ADC_MIN = 0, ADC_MAX = 4095;
constexpr int DEAD_LOW = 1750, DEAD_HIGH = 2150;
static_assert(ADC_MIN < DEAD_LOW && DEAD_LOW < DEAD_HIGH && DEAD_HIGH < ADC_MAX,
              "Invalid joystick calibration");

inline int steeringAngle(int raw) {
  if (raw < ADC_MIN) raw = ADC_MIN;
  if (raw > ADC_MAX) raw = ADC_MAX;
  if (raw < DEAD_LOW) return 60 + (raw - ADC_MIN) * 30 / (DEAD_LOW - ADC_MIN);
  if (raw > DEAD_HIGH) return 90 + (raw - DEAD_HIGH) * 30 / (ADC_MAX - DEAD_HIGH);
  return 90;
}
