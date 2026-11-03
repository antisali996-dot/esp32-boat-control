#pragma once
#include <cstdint>
#include <cstring>

// 编码器累计计数 + 测速窗口的原子快照（纯逻辑，可在电脑上用 g++ 回归）。
//
// 约定：
//   - totalPulses 与窗口计数在同一个 ISR 临界区内一起递增；
//   - totalPulses 只在构造（上电）时为 0，窗口清零绝不动它；
//   - 主循环读取 64 位累计值时，必须整体取快照，不能分两次读高低 32 位。
//
// 段内容（临界区内只允许做这些）：
//   - 读时间 → 读窗口计数 → 读 total → 清窗口计数 → 更新基准
// 浮点换算、格式化、日志投递一律在临界区外。

struct PulseSnapshot {
  uint32_t sampleMs = 0;
  uint32_t windowPulses = 0;
  uint64_t totalPulses = 0;
  bool valid = false;
};

struct PulseCounter {
  // ISR 与主循环共享；由调用者用同一把锁保护。
  uint32_t windowPulses = 0;
  uint64_t totalPulses = 0;     // 仅上电初始化为 0

  // ISR 路径：同一个临界区内一起递增。
  void onEdge() {
    ++windowPulses;
    ++totalPulses;
  }

  // 窗口交换：同一临界区内读取时间/窗口/total，并清零窗口、更新基准。
  // 返回的 sampleMs 与两个计数来自同一快照。
  PulseSnapshot takeWindow(uint32_t nowMs, uint32_t *baselineMsOut) {
    PulseSnapshot s;
    s.sampleMs = nowMs;
    s.windowPulses = windowPulses;
    s.totalPulses = totalPulses;
    s.valid = true;
    windowPulses = 0;
    if (baselineMsOut) *baselineMsOut = nowMs;
    return s;
  }

  // 只读快照（JAY COUNT 用）：不改任何计数。
  PulseSnapshot peek(uint32_t nowMs) const {
    PulseSnapshot s;
    s.sampleMs = nowMs;
    s.windowPulses = windowPulses;
    s.totalPulses = totalPulses;
    s.valid = true;
    return s;
  }
};


// 建立测速基准：固件首次初始化与电脑测试共用这一个函数。
//
// 调用者必须已持有 stateMux（本函数不加锁，也不依赖 Arduino）。
// 只清窗口计数，绝不动 totalPulses。
// 返回的 sampleMs / totalPulses 来自清零后、同一时刻的快照。
inline PulseSnapshot beginBaseline(PulseCounter &c, uint32_t t0Ms) {
  c.windowPulses = 0;          // 唯一要清的东西
  return c.peek(t0Ms);         // total 保持累计值
}

// 启动标识：16 字节随机数编码成 32 个十六进制字符，本次启动期间不变。
// 只用于区分诊断会话，不作为安全凭据，也不写入 NVS。
struct BootId {
  uint8_t raw[16] = {0};
  char hex[33] = {0};

  static char nibbleToHex(uint8_t v) {
    return (char)(v < 10 ? ('0' + v) : ('a' + (v - 10)));
  }

  // filler 负责把 16 字节填满（生产用 esp_fill_random，测试用受控数据）。
  void generate(void (*filler)(uint8_t *, size_t)) {
    filler(raw, sizeof(raw));
    encode();
  }

  void encode() {
    for (int i = 0; i < 16; ++i) {
      hex[i * 2] = nibbleToHex(raw[i] >> 4);
      hex[i * 2 + 1] = nibbleToHex(raw[i] & 0x0F);
    }
    hex[32] = 0;
  }

  bool valid() const { return hex[0] != 0 && strlen(hex) == 32; }
};
