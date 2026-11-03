#pragma once
#include <stdint.h>

// 遥控发送状态机（纯逻辑，不依赖 Arduino，可在电脑上用 g++ 直接回归测试）。
//
// 状态流转：
//   INIT ──radio up──> READY ──requestSend──> WAIT_DONE
//     ^                  ^                        │
//     │                  │                  回调/本地错误
//     │                  └────────────────────────┘
//     │                  │
//     │            回调超时（先禁止新发送）
//     │                  v
//     └──────────── BACKOFF ──等满退避──> REBUILD ──成功──> READY
//                                          │
//                                          └失败×3──> FAULT（明确故障，不再快速重试）
//
// 关于迟到回调：
//   仅靠一个全局 generation 变量并不构成证明。本实现依赖两点共同成立：
//   (1) 核心侧：mt_unregister_send_cb 会把回调指针清零，mt_deinit 也会清零，
//       而 mt_send_cb 在调用前重新读取该指针，因此注销之后新触发的回调不会进入本代码
//       ——这一点已由本地 libespnow.a 反汇编核实（见 docs/re-basis）；
//   (2) 本侧：pending 的结束只接受“令牌匹配”的回调，令牌修改全程在临界区内完成。
//       注销瞬间已经在回调内、尚未取到锁的旧回调，拿到锁后会看到令牌失配而被丢弃，
//       不会错误地结束新一轮发送。
//   仍需实测确认：注销/deinit 期间是否还有已进入核心、越过空指针判断的回调。
//   在该窗口被实测排除之前，迟到回调的彻底隔离只能视为“设计上已处理、待实测”。

struct TxStateMachine {
  // 与固件需求一致的固定退避序列（毫秒）。
  static constexpr uint32_t BACKOFF_MS[3] = {500, 1000, 2000};
  static constexpr uint8_t MAX_REBUILD_FAILURES = 3;
  static constexpr uint32_t SEND_TIMEOUT_MS = 500;

  enum State { INIT, READY, WAIT_DONE, BACKOFF, REBUILD, FAULT };

  State state = INIT;
  uint32_t pendingToken = 0;   // 0 表示当前没有在途发送
  uint32_t nextToken = 1;
  uint32_t sentAt = 0;
  uint32_t backoffUntil = 0;

  uint8_t consecutiveRebuildFailures = 0;
  uint32_t backoffIndex = 0;

  // 三类失败分别计数，便于区分问题来源。
  uint32_t localSendErrors = 0;   // esp_now_send 立即返回错误
  uint32_t sendFailCallbacks = 0; // 回调报告对端未收到
  uint32_t callbackTimeouts = 0;  // 回调超时（对端完全无响应）
  uint32_t delivered = 0;
  uint32_t radioRebuilds = 0;
  uint32_t rebuildFailures = 0;

  // 重建成功后的静默窗口：先不发送，让任何“已进入核心、尚未返回”的旧回调先落地。
  // 这不是“已证明迟到回调不会串台”的等价物，只是把窗口压小；见文件头说明。
  static constexpr uint32_t REBUILD_QUIET_MS = 100;
  uint32_t readyAt = 0;
  // 用独立标志而非 readyAt==0 作哨兵：跨时钟回绕时 0 会被误判成未来的截止点。
  bool quietActive = false;

  bool faultLatched() const { return state == FAULT; }

  // 无符号回绕安全的“时间点已到/已过”判定。
  // now == deadline 时差值为 0，按“已到”返回 true。
  static bool reached(uint32_t now, uint32_t deadline) {
    const uint32_t diff = now - deadline;
    return diff < 0x80000000u;
  }

  // 无线初始化成功（或有有效对端）后调用。
  void onRadioUp() {
    if (state == FAULT) return;
    state = READY;
    pendingToken = 0;
    consecutiveRebuildFailures = 0;
    backoffIndex = 0;
    readyAt = 0;
    quietActive = false;
  }

  void onRadioDown() {
    if (state == FAULT) return;
    state = INIT;
    pendingToken = 0;
  }

  bool canSend(uint32_t now) const {
    return state == READY && pendingToken == 0 &&
           (!quietActive || reached(now, readyAt));
  }

  // 开始一次发送；返回本次发送令牌（非 0 表示已登记在途）。
  // 调用者必须只在返回非 0 时才真正调用 esp_now_send。
  uint32_t beginSend(uint32_t now) {
    if (!canSend(now)) return 0;
    pendingToken = nextToken++;
    if (nextToken == 0) nextToken = 1; // 0 保留为“无在途”
    sentAt = now;
    state = WAIT_DONE;
    return pendingToken;
  }

  // esp_now_send 立即返回错误：本地错误，清在途，回到可发送，不重启无线。
  void onLocalSendError() {
    ++localSendErrors;
    pendingToken = 0;
    if (state == WAIT_DONE) state = READY;
  }

  // 发送完成回调。token 不匹配（迟到回调）时完全忽略。
  void onSendCallback(uint32_t token, bool success) {
    if (pendingToken == 0 || token != pendingToken) return; // 迟到回调，丢弃
    pendingToken = 0;
    if (success) ++delivered;
    else ++sendFailCallbacks; // 普通“对端未收到”，不重启无线
    if (state == WAIT_DONE) state = READY;
  }

  static uint32_t backoffMs(uint32_t index) {
    return BACKOFF_MS[index < 3 ? index : 2];
  }

  // 周期性推进。返回 true 表示调用者现在应当执行“重建无线”的副作用：
  // 先 esp_now_unregister_send_cb()，再 esp_now_deinit()，再重新 init/注册/加对端。
  bool tick(uint32_t now) {
    if (state == WAIT_DONE && uint32_t(now - sentAt) >= SEND_TIMEOUT_MS) {
      ++callbackTimeouts;
      pendingToken = 0;         // 先禁止新发送（state 立刻离开可为发送的状态）
      backoffIndex = 0;
      backoffUntil = now + backoffMs(0);
      state = BACKOFF;
    }
    if (state == BACKOFF && reached(now, backoffUntil)) {
      state = REBUILD;
      return true;
    }
    return false;
  }

  // 重建结果由调用者回报。
  void onRebuildResult(bool ok, uint32_t now) {
    if (state != REBUILD) return;
    ++radioRebuilds;
    if (ok) {
      consecutiveRebuildFailures = 0;
      backoffIndex = 0;
      pendingToken = 0;
      readyAt = now + REBUILD_QUIET_MS;
      quietActive = true;
      state = READY;
      return;
    }
    ++rebuildFailures;
    ++consecutiveRebuildFailures;
    if (consecutiveRebuildFailures >= MAX_REBUILD_FAILURES) {
      state = FAULT;   // 明确故障，不再尝试
      pendingToken = 0;
      return;
    }
    backoffIndex = consecutiveRebuildFailures;
    backoffUntil = now + backoffMs(backoffIndex);
    state = BACKOFF;
  }

  const char *stateName() const {
    switch (state) {
      case INIT: return "INIT";
      case READY: return "READY";
      case WAIT_DONE: return "WAIT_DONE";
      case BACKOFF: return "BACKOFF";
      case REBUILD: return "REBUILD";
      case FAULT: return "FAULT";
    }
    return "?";
  }
};
