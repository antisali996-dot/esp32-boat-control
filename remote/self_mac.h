#pragma once
#include <cstring>

#include "self_mac_logic.h"

// 自身 STA 地址的读取入口（纯逻辑，不依赖 Arduino）。
//
// 为什么不用 WiFi.macAddress()：
//   1. 未配对时 WiFi 可能还没启动，WiFi.macAddress() 会返回全零或未定义值；
//   2. 全零 MAC 会让身份查询行看起来像“未配对”，也会污染自身地址比较与配对校验。
//
// 真实读取函数由调用者注入：
//   - 固件在 firmware_port_arduino.h 里注入 esp_read_mac(ESP_MAC_WIFI_STA)；
//   - 电脑测试注入受控桩。
// 判定逻辑（成功且非全零才算可用）在 self_mac_logic.h，两边共用同一份。

struct SelfMac {
  SelfMacState state;

  // reader 返回 0 表示成功（与 esp_err_t 的 ESP_OK 一致）。
  bool read(int (*reader)(uint8_t *out)) { return state.read(reader); }

  bool ok() const { return state.ok; }
  int status() const { return state.status; }
  const uint8_t *addr() const { return state.addr; }
  bool equals(const uint8_t *other) const { return state.equals(other); }
};
