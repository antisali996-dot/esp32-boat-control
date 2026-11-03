#pragma once
#include <stdint.h>

// 遥控发送状态机（纯逻辑，不依赖 Arduino，可在电脑上用 g++ 直接回归测试）。
//
// 状态流转：
//   INIT ──radio up──> READY ──beginSend──> WAIT_DONE
//                        ^                     │
//                        │                回调（成功/失败）
//                        └─────────────────────┘
//                                              │
//                                        回调超时
//                                              v
//                                            FAULT（锁存，需重新上电）
//
// 为什么回调超时直接进 FAULT，而不是自动退避重建：
//   ESP-NOW 的发送回调没有请求编号，只能看到“某个发送完成了”。
//   deinit 也不等待已经进入核心的在途发送。因此“超时后重建无线再接着发”
//   无法证明旧回调不会在重建后落地；即使加了令牌，也只能丢弃事件，
//   不能证明底层状态已经干净。
//   本实现选择不做自动恢复：超时即锁存 FAULT，停发，等人工断电重启。
//   这是临时隔离措施，不是可靠的自动恢复方案。
//
// 迟到回调：FAULT 内以及没有在途发送时到达的回调一律丢弃（只计数，不推进状态）。
//   令牌只用于“同一时刻只允许一个在途发送”的基本互斥，不承担跨重建区分的新旧语义。

struct TxStateMachine {
  static constexpr uint32_t SEND_TIMEOUT_MS = 500;

  enum State { INIT, READY, WAIT_DONE, FAULT };

  State state = INIT;
  uint32_t pendingToken = 0;   // 0 表示当前没有在途发送
  uint32_t nextToken = 1;
  uint32_t sentAt = 0;

  // 三类失败分别计数，便于区分问题来源。
  uint32_t localSendErrors = 0;   // esp_now_send 立即返回错误（偶发，可继续）
  uint32_t sendFailCallbacks = 0; // 回调报告对端未收到（普通丢包，继续按原策略发送）
  uint32_t callbackTimeouts = 0;  // 回调超时（锁存 FAULT）
  uint32_t delivered = 0;
  uint32_t staleCallbacks = 0;    // 无在途/FAULT 时到达的迟到回调（仅统计）

  bool faultLatched() const { return state == FAULT; }

  void onRadioUp() {
    if (state == FAULT) return;
    state = READY;
    pendingToken = 0;
  }

  void onRadioDown() {
    if (state == FAULT) return;
    state = INIT;
    pendingToken = 0;
  }

  // 可以发起新发送：必须处于 READY 且没有在途发送。
  bool canSend(uint32_t) const { return state == READY && pendingToken == 0; }

  // 开始一次发送；返回本次发送令牌（非 0 表示已登记在途）。
  // 调用者只在返回非 0 时才真正调用 esp_now_send。
  uint32_t beginSend(uint32_t now) {
    if (!canSend(now)) return 0;
    pendingToken = nextToken++;
    if (nextToken == 0) nextToken = 1; // 0 保留为“无在途”
    sentAt = now;
    state = WAIT_DONE;
    return pendingToken;
  }

  // esp_now_send 立即返回错误：本地错误，清在途，回到可发送，不重启无线，不锁存。
  void onLocalSendError() {
    ++localSendErrors;
    pendingToken = 0;
    if (state == WAIT_DONE) state = READY;
  }

  // 发送完成回调（由适配层在临界区内调用）。
  // FAULT 内、以及没有在途发送时到达的回调全部丢弃并计数。
  void onSendCallback(uint32_t token, bool success) {
    if (state == FAULT) { ++staleCallbacks; return; }
    if (pendingToken == 0 || token != pendingToken) { ++staleCallbacks; return; }
    pendingToken = 0;
    if (success) ++delivered;
    else ++sendFailCallbacks; // 普通“对端未收到”：不重启、不锁存
    if (state == WAIT_DONE) state = READY;
  }

  // 周期性检查超时。返回 true 表示刚刚因超时锁存 FAULT（调用者据此打印一次）。
  bool tick(uint32_t now) {
    if (state == WAIT_DONE && uint32_t(now - sentAt) >= SEND_TIMEOUT_MS) {
      ++callbackTimeouts;
      // 原子地清空全部在途痕迹，再锁存。不自动重建、不自动重新发送。
      pendingToken = 0;
      state = FAULT;
      return true;
    }
    return false;
  }

  const char *stateName() const {
    switch (state) {
      case INIT: return "INIT";
      case READY: return "READY";
      case WAIT_DONE: return "WAIT_DONE";
      case FAULT: return "FAULT";
    }
    return "?";
  }
};
