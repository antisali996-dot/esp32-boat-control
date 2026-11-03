#pragma once
#include <cstring>

#include "firmware_port.h"
#include "fw_role.h"
#include "identity_line.h"
#include "pair_mac.h"
#include "run_log.h"
#include "self_mac.h"

// USB 配对命令处理（JAY INFO / JAY PING / JAY BIND）。
//
// 关键点：保存成功但回应没能送出时，错误不能只存在于（可能已满的）日志队列里。
// 这里用一个独立的“待发送错误”状态保存它，并在之后每次处理命令前重试投递；
// 投递成功后还要等该序号确认 println+flush 完成，才清除状态。
// 这个错误的发送完成绝不触发重启——只有“保存成功 + 回应确认送出”的正规路径才重启。
//
// 全部依赖通过 FirmwarePort 注入：测试执行的就是这段代码本身。

// 待发送错误码（仅内部状态；实际发出去的是下面的文本）
enum PendingPairError {
  PENDING_NONE = 0,
  PENDING_ACK_NOT_QUEUED = 1,   // 保存回应未能入队
  PENDING_ACK_UNCONFIRMED = 2,  // 保存回应入队但未确认送出
};

// 配对模块的全部可变状态。固件里是单个静态实例，测试里可以独立构造。
struct PairingState {
  // ---- 待发送错误（独立于日志队列） ----
  PendingPairError pending = PENDING_NONE;
  bool pendingInFlight = false;      // 已入队、等待确认完成
  uint32_t pendingSeq = 0;           // 该条错误的序号
  uint32_t pendingRequeueAttempts = 0;
  uint32_t pendingResolvedCount = 0; // 已确认送出并清除的次数

  // ---- 统计 ----
  uint32_t savedButUnacked = 0;      // 保存成功但回应未确认的次数
  uint32_t rejectedSecondBind = 0;   // 因存在未处理错误而拒绝的第二次绑定

  bool hasUnresolvedError() const { return pending != PENDING_NONE; }

  static const char *pendingText(PendingPairError e) {
    switch (e) {
      case PENDING_ACK_NOT_QUEUED: return "JAY ERROR=SAVED_ACK_NOT_QUEUED";
      case PENDING_ACK_UNCONFIRMED: return "JAY ERROR=SAVED_ACK_UNCONFIRMED";
      default: return "";
    }
  }

  void raise(PendingPairError e) {
    // 已有更严重的未处理错误时不覆盖，避免丢失最早的失败证据。
    if (pending == PENDING_NONE) {
      pending = e;
      pendingInFlight = false;
      pendingSeq = 0;
      pendingRequeueAttempts = 0;
    }
  }

  // 在每次处理命令前调用：零等待尝试投递待发送错误。
  // 队列仍满就保留状态，不循环、不阻塞、不重复入队。
  void pumpPending(RunLog &log) {
    if (pending == PENDING_NONE) return;
    if (pendingInFlight) {
      // 已入队：只等之前那条的确认，绝不重复入队。
      if (log.logWaitDone(pendingSeq, 0)) {
        pending = PENDING_NONE;
        pendingInFlight = false;
        pendingSeq = 0;
        ++pendingResolvedCount;
      }
      return;
    }
    uint32_t seq = 0;
    const FwLogPost r = log.logCritical(&seq, "%s", pendingText(pending));
    ++pendingRequeueAttempts;
    if (r == FW_LOG_OK) {
      pendingInFlight = true;
      pendingSeq = seq;
      // 立即检查一次（不等待）：若刚好已完成，下一轮会清除。
    }
    // 失败（队列满/禁用/格式化错误）：保留 pending，等下次命令前再试。
  }
};

// 把身份主行复制到调用者缓冲；格式化失败时返回 false（不发送半行）。
inline bool outTextOrEmpty(char *out, size_t cap, const IdentityLines &lines) {
  if (!lines.mainOk()) { if (cap) out[0] = 0; return false; }
  strncpy(out, lines.main, cap - 1);
  out[cap - 1] = 0;
  return true;
}

// 启动横幅里调用一次：输出身份行（与 JAY INFO 相同的格式串）。
inline void pairInfo(const char *role, const uint8_t *peer, const SelfMac &self, RunLog &log) {
  const IdentityLines lines = buildIdentityLines(role, self.addr(), self.ok(), peer,
                                                 FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                                 FW_ID, FW_VER);
  if (!lines.mainOk()) return;
  log.logCritical(nullptr, "%s", lines.main);
  if (lines.selfMacValid && lines.legacyOk()) log.logCritical(nullptr, "%s", lines.legacy);
}

// 只读诊断钩子：小船实现 JAY COUNT / JAY FAULT；遥控器传 nullptr（不伪造编码器数据）。
// 返回 true 表示该命令已被处理并已投递回应。
struct DiagHooks {
  bool (*handleCount)(RunLog &log) = nullptr;
  bool (*handleFault)(RunLog &log) = nullptr;
};

// 返回：这条命令是否已经完整处理（用于测试断言）。
// peer 为当前运行期配对值（不会被本函数在失败路径改写）。
inline void servicePairing(const char *role, uint8_t *peer, const SelfMac &self,
                           RunLog &log, FirmwarePort &port, PairingState &state,
                           const DiagHooks &diag = DiagHooks{}) {
  // 处理任何命令之前，先尝试补发上一条未处理的保存错误。
  state.pumpPending(log);

  static char line[96];
  static size_t len = 0;
  static bool overflow = false;

  while (true) {
    // 从串口读一行。抽取到 port 之外以保持可测试性：这里用 port 提供的非阻塞读。
    int c = -1;
    if (!port.serialRead(&c)) break;
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(line) - 1 && !overflow) line[len++] = (char)c;
      else overflow = true;
      continue;
    }
    line[len] = 0;
    len = 0;

    if (overflow) {
      overflow = false;
      log.logCritical(nullptr, "JAY ERROR=LINE_TOO_LONG");
      continue;
    }

    if (strcmp(line, "JAY INFO") == 0) {
      char text[IDENTITY_LINE_CAP];
      const IdentityLines lines = buildIdentityLines(role, self.addr(), self.ok(), peer,
                                                     FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                                     FW_ID, FW_VER);
      if (outTextOrEmpty(text, sizeof(text), lines)) {
        uint32_t seq = 0;
        if (log.logCritical(&seq, "%s", text) == FW_LOG_OK) {
          log.logWaitDone(seq, 200);
        }
        if (lines.selfMacValid && lines.legacyOk()) {
          log.logCritical(nullptr, "%s", lines.legacy);
        }
      }
      continue;
    }

    if (strcmp(line, "JAY PING") == 0) {
      if (self.ok()) log.logCritical(nullptr, "JAY PONG FWID=%s VER=%s", FW_ID, FW_VER);
      else log.logCritical(nullptr, "JAY PONG FWID=%s VER=%s SELF_MAC=READ_FAILED", FW_ID, FW_VER);
      continue;
    }

    // 只读诊断：不要求配对、不要求 Wi-Fi，不改 NVS/计数/控制状态。
    if (strcmp(line, "JAY COUNT") == 0) {
      if (diag.handleCount) diag.handleCount(log);
      else log.logCritical(nullptr, "JAY ERROR=UNSUPPORTED_COMMAND");
      continue;
    }

    if (strcmp(line, "JAY FAULT") == 0) {
      if (diag.handleFault) diag.handleFault(log);
      else log.logCritical(nullptr, "JAY ERROR=UNSUPPORTED_COMMAND");
      continue;
    }

    if (strncmp(line, "JAY BIND ", 9) == 0) {
      // 存在未处理错误时拒绝第二条绑定，保护第一条失败证据。
      if (state.hasUnresolvedError()) {
        ++state.rejectedSecondBind;
        log.logCritical(nullptr, "JAY ERROR=BIND_BLOCKED_PENDING_ERROR");
        continue;
      }
      uint8_t replacement[6];
      if (!self.ok()) {
        log.logCritical(nullptr, "JAY ERROR=SELF_MAC_READ_FAILED");
        continue;
      }
      if (!parsePeerMac(line + 9, replacement) || self.equals(replacement)) {
        log.logCritical(nullptr, "JAY ERROR=INVALID_PEER");
        continue;
      }
      if (!port.nvsSavePeer(replacement)) {
        log.logCritical(nullptr, "JAY ERROR=SAVE_FAILED");
        continue;
      }
      // NVS 已写入。从现在起，任何失败都不能重启，且不能改运行期配对值。
      uint32_t seq = 0;
      const FwLogPost r = log.logCritical(&seq, "JAY SAVED REBOOT");
      if (r != FW_LOG_OK) {
        ++state.savedButUnacked;
        state.raise(PENDING_ACK_NOT_QUEUED);
        continue;
      }
      if (!log.logWaitDone(seq, 200)) {
        ++state.savedButUnacked;
        state.raise(PENDING_ACK_UNCONFIRMED);
        continue;
      }
      port.restart();     // 唯一允许的重启点
      continue;
    }

    log.logCritical(nullptr, "JAY ERROR=UNKNOWN_COMMAND");
  }
}
