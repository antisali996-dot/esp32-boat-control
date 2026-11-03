#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

// 自身 MAC 读取的纯逻辑部分：不依赖 Arduino，便于在电脑上覆盖三种情况：
//   1. 空 NVS / 未配对 —— 与 MAC 读取无关，读取仍然成功；
//   2. 无线未初始化 —— esp_read_mac 不经过 WiFi 栈，读取仍然成功；
//   3. 读取失败 —— 返回错误码，调用者必须能区分“失败”和“全零”。
//
// 真实实现把 esp_read_mac 作为函数指针注入（见 self_mac.h），
// 测试则注入受控的假实现，因此测试的是同一份判定代码。

// 与 esp_err_t 的取值保持一致，避免在纯逻辑文件里引入 esp_err.h。
constexpr int SELF_MAC_OK = 0;
constexpr int SELF_MAC_INVALID_ARG = 0x102;   // ESP_ERR_INVALID_ARG
constexpr int SELF_MAC_INVALID_MAC = 0x10A;   // ESP_ERR_INVALID_MAC

struct SelfMacState {
  uint8_t addr[6] = {0, 0, 0, 0, 0, 0};
  int status = -1;      // -1 = 尚未读取
  bool ok = false;      // 读取成功且非全零

  // reader 返回 0 表示成功，其它值为错误码。
  // 判定规则：只有“返回成功”且“结果非全零”才算拿到可用地址。
  bool read(int (*reader)(uint8_t *out)) {
    uint8_t raw[6] = {0, 0, 0, 0, 0, 0};
    status = reader(raw);
    bool nonzero = false;
    for (int i = 0; i < 6; ++i) nonzero |= raw[i] != 0;
    ok = (status == SELF_MAC_OK) && nonzero;
    if (ok) std::memcpy(addr, raw, sizeof(addr));
    else std::memset(addr, 0, sizeof(addr));
    return ok;
  }

  // 读取失败时给出 "READ_FAILED"，绝不输出 00:00:00:00:00:00 冒充合法地址。
  void format(char *out, unsigned long outLen) const {
    if (!ok) {
      std::snprintf(out, outLen, "READ_FAILED");
      return;
    }
    std::snprintf(out, outLen, "%02X:%02X:%02X:%02X:%02X:%02X",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  }

  // 自身地址是否与候选对端相同。读取失败时返回 false（无法判定，交给调用者拒绝写入）。
  bool equals(const uint8_t *other) const {
    if (!ok) return false;
    return std::memcmp(addr, other, 6) == 0;
  }
};
