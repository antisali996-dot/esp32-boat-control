#pragma once
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "firmware_port.h"
#include "identity_line.h"

// 日志核心：普通/关键两条通道、序号分配、完成确认。
//
// 这里只保留逻辑，实际的队列、串口、flush、时钟全部通过 FirmwarePort 注入。
// 生产环境见 firmware_port_arduino.h；测试环境在 tests 里绑定受控桩。
// 因此电脑测试执行的就是固件里这套 logCritical / logWaitDone 代码本身，
// 不存在“测试另写一个模型去代替固件”的情况。

constexpr int RUN_LOG_QUEUE_DEPTH = 8;
constexpr int RUN_LOG_CRITICAL_DEPTH = 4;
constexpr size_t RUN_LOG_LINE_MAX = IDENTITY_LINE_CAP;   // 256

struct RunLogEntry {
  char text[RUN_LOG_LINE_MAX];
  uint32_t seq;
};

// 关键行的投递结果沿用 firmware_port.h 的 FwLogPost。

struct RunLog {
  FirmwarePort *port = nullptr;
  bool enabled = false;
  uint32_t seqCounter = 0;       // 已分配的最大序号

  void begin(FirmwarePort *p) { port = p; enabled = (p != nullptr); }

  // 格式化并检查截断。返回 FMT_OK 才允许发送。
  static FmtStatus format(char *out, size_t cap, const char *fmt, va_list args) {
    const int n = vsnprintf(out, cap, fmt, args);
    return checkFmt(n, cap);
  }

  // 普通运行日志：零等待；截断/失败只计数。
  void logLine(const char *fmt, ...) {
    if (!enabled) return;
    char buf[RUN_LOG_LINE_MAX];
    va_list args;
    va_start(args, fmt);
    const FmtStatus st = format(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (st != FMT_OK) return;                  // 由 port 统计 fmtErrors
    port->normalPost(buf);
  }

  // 关键协议行：零等待投递，返回明确结果与序号。
  FwLogPost logCritical(uint32_t *seqOut, const char *fmt, ...) {
    if (seqOut) *seqOut = 0;
    if (!enabled) return FW_LOG_DISABLED;
    char buf[RUN_LOG_LINE_MAX];
    va_list args;
    va_start(args, fmt);
    const FmtStatus st = format(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (st != FMT_OK) return FW_LOG_FMT_ERROR;   // 明确失败，不发送半行

    uint32_t seq = 0;
    const FwLogPost r = port->criticalPost(buf, &seq);
    if (r == FW_LOG_OK) {
      if (seq > seqCounter) seqCounter = seq;
      if (seqOut) *seqOut = seq;
    }
    return r;
  }

  // 等待指定序号真正 println + flush 完成。
  // 只允许 USB 维护命令路径调用，不得出现在控制路径。
  bool logWaitDone(uint32_t seq, uint32_t timeoutMs) {
    if (!enabled || seq == 0) return false;
    const uint32_t start = port->nowMs();
    for (;;) {
      if (port->seqDone(seq)) return true;
      if (uint32_t(port->nowMs() - start) >= timeoutMs) return false;
      port->sleepMs(1);   // 只让出 CPU，不作为“完成”的证据
    }
  }
};
