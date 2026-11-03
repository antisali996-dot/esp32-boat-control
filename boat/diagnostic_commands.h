#pragma once
#include <cstdint>
#include <cstdio>
#include <inttypes.h>

#include "fault_evidence.h"
#include "firmware_port.h"
#include "identity_line.h"
#include "pulse_count.h"
#include "run_log.h"

// 小船只读诊断命令的共享实现（COUNT / FAULT）。
//
// 固件与电脑测试调用的是同一份代码：boat.ino 里只剩“取快照 → 交给这里”的薄包装，
// 不再保留第二套格式串、入队判断或布尔转换。
//
// 不依赖 Arduino：只用到 RunLog（真实日志核心）与 FaultEvidence/PulseSnapshot（纯逻辑）。
//
// 返回值语义：true 仅表示“本命令所需的消息全部成功入队”，
// 不代表已经发送完成。logCritical 返回 FwLogPost 枚举，
// FW_LOG_OK 的值为 0，因此一律显式比较，绝不直接当 bool。

namespace diag {

// 投递一行；格式化失败或入队失败都返回 false（不发送截断半行）。
inline bool postFormatted(RunLog &log, const char *fmt, ...) {
  char buf[IDENTITY_LINE_CAP];
  va_list args;
  va_start(args, fmt);
  const FmtStatus st = RunLog::format(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (st != FMT_OK) return false;

  uint32_t seq = 0;
  return log.logCritical(&seq, "%s", buf) == FW_LOG_OK;
}

// JAY COUNT：调用者已在临界区内取好快照（时间与 total 同源）。
inline bool handleCount(RunLog &log, const char *bootHex, const PulseSnapshot &snap) {
  return postFormatted(log, "JAY COUNT boot=%s ms=%lu total=%" PRIu64,
                       bootHex, (unsigned long)snap.sampleMs, snap.totalPulses);
}

// JAY FAULT：无故障回 NONE；有故障回固定两行，id 相同。
// 第一行失败时不再投递第二行，避免只送出半对；锁存证据不受影响。
inline bool handleFault(RunLog &log, const FaultEvidence &ev) {
  if (!ev.latched) return postFormatted(log, "JAY FAULT NONE");

  char cur[IDENTITY_LINE_CAP];
  char prev[IDENTITY_LINE_CAP];
  if (!FaultEvidence::formatCurrent(cur, sizeof(cur), ev)) return false;
  if (!FaultEvidence::formatPrevious(prev, sizeof(prev), ev)) return false;

  uint32_t seq = 0;
  if (log.logCritical(&seq, "%s", cur) != FW_LOG_OK) return false;
  if (log.logCritical(&seq, "%s", prev) != FW_LOG_OK) return false;
  return true;
}

}  // namespace diag
