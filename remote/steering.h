#pragma once

// 角度范围与死区均取自 fw_identity.h，保证身份串与实际映射同源。
#include "fw_identity.h"

// 保留原程序死区；摇杆实际端点应通过串口读数校准。
constexpr int ADC_MIN = 0, ADC_MAX = 4095;
constexpr int DEAD_LOW = FW_JOY_DEAD_LOW, DEAD_HIGH = FW_JOY_DEAD_HIGH;
static_assert(ADC_MIN < DEAD_LOW && DEAD_LOW < DEAD_HIGH && DEAD_HIGH < ADC_MAX,
              "Invalid joystick calibration");
static_assert(FW_MIN_ANGLE < FW_CENTER_ANGLE && FW_CENTER_ANGLE < FW_MAX_ANGLE,
              "Invalid angle range");

inline int steeringAngle(int raw) {
  if (raw < ADC_MIN) raw = ADC_MIN;
  if (raw > ADC_MAX) raw = ADC_MAX;
  // 死区两侧分段映射，端点分别落在 FW_MIN_ANGLE / FW_MAX_ANGLE。
  if (raw < DEAD_LOW) return FW_MIN_ANGLE + (raw - ADC_MIN) * FW_STEER_DEG / (DEAD_LOW - ADC_MIN);
  if (raw > DEAD_HIGH) return FW_CENTER_ANGLE + (raw - DEAD_HIGH) * FW_STEER_DEG / (ADC_MAX - DEAD_HIGH);
  return FW_CENTER_ANGLE;
}
