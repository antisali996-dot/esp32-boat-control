#pragma once
#include <cstdint>

// ESP-NOW 发送回调的适配层（纯逻辑版）。
//
// 存在的理由：
//   ESP-NOW 的发送完成回调没有请求编号，只有“某次发送结束了”。
//   如果直接把“当前 inFlightToken”贴在回调上，迟到的旧回调会被误认为
//   是在途的新发送，从而错误地结束它。这里把判定集中到一处：
//   只有“确实存在在途发送”时，回调才被采纳并立刻吊销在途标记。
//
// 固件与电脑测试共用这一份代码（remote/remote.ino 的 OnDataSent 调用它），
// 因此测试走的是真实适配层，而不是向状态机手工注入旧 token。

enum TxCallbackKind {
  TX_CB_NONE = -1,      // 没有事件
  TX_CB_FAILED = 0,     // 对应在途发送的失败回调
  TX_CB_SUCCESS = 1,    // 对应在途发送的成功回调
  TX_CB_STALE = 2,      // 没有在途发送（迟到回调）
};

struct TxCallbackPort {
  uint32_t inFlightToken = 0;   // 0 = 无在途
  int32_t pendingCallback = TX_CB_NONE;
  uint32_t callbackToken = 0;

  // 在途登记：只由控制循环调用。
  void arm(uint32_t token) { inFlightToken = token; }

  // 吊销在途（本地发送错误、超时清场）。
  void disarm() { inFlightToken = 0; }

  // 回调入口（真实实现里整段在临界区内执行）。
  // 返回本次回调被判定成什么，便于测试直接断言分类结果。
  TxCallbackKind onSendCallback(bool success) {
    const uint32_t token = inFlightToken;
    if (token != 0) {
      pendingCallback = success ? TX_CB_SUCCESS : TX_CB_FAILED;
      callbackToken = token;
      inFlightToken = 0;          // 同一次发送的回调只被采纳一次
      return (TxCallbackKind)pendingCallback;
    }
    // 没有在途发送：迟到回调。不携带任何 token，状态机只会计数后丢弃。
    pendingCallback = TX_CB_STALE;
    callbackToken = 0;
    return TX_CB_STALE;
  }

  // 控制循环取走事件。
  TxCallbackKind take(uint32_t *tokenOut) {
    const TxCallbackKind kind = (TxCallbackKind)pendingCallback;
    if (tokenOut) *tokenOut = callbackToken;
    pendingCallback = TX_CB_NONE;
    return kind;
  }
};
