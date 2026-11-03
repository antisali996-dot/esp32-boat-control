#pragma once
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "../boat/firmware_port.h"
#include "../boat/run_log.h"

// 受控端口桩：只实现“平台能力”，不含任何业务逻辑。
// 固件的 run_log.h / pairing.h 是唯一实现业务的地方，测试直接驱动它们。

struct StubPort : public FirmwarePort {
  // ---- 队列 ----
  std::deque<std::pair<uint32_t, std::string>> criticalQ;
  std::deque<std::string> normalQ;
  int criticalCap = RUN_LOG_CRITICAL_DEPTH;
  int normalCap = RUN_LOG_QUEUE_DEPTH;

  // 强制投递失败（模拟队列满），与容量无关，便于精确构造场景。
  bool forceCriticalFull = false;

  // ---- 串口输出 ----
  std::vector<std::string> written;      // 已 println 的行
  std::vector<std::string> flushedAfter;  // 每次 flush 前最后写出的行

  // ---- 完成状态 ----
  uint32_t nextSeq = 1;
  uint32_t lastDone = 0;
  // 取走但不发布完成（模拟“已从队列取出、println/flush 还没完成”）。
  bool holdDelivery = false;

  // ---- 时钟 ----
  uint32_t clockMs = 0;
  uint32_t sleepAdvanceMs = 1;   // sleepMs 推进的时钟

  // ---- NVS ----
  uint8_t savedPeer[6] = {0, 0, 0, 0, 0, 0};
  int nvsWrites = 0;
  bool nvsResult = true;

  // ---- 重启 ----
  int restartCount = 0;

  // ---- 输入 ----
  std::string input;

  // ---- 计数 ----
  uint32_t droppedNormal_ = 0;
  uint32_t droppedCritical_ = 0;
  uint32_t fmtErrors_ = 0;

  // ===== FirmwarePort =====
  FwLogPost criticalPost(const char *line, uint32_t *seqOut) override {
    if (seqOut) *seqOut = 0;
    if (forceCriticalFull || (int)criticalQ.size() >= criticalCap) {
      ++droppedCritical_;
      return FW_LOG_QUEUE_FULL;
    }
    const uint32_t seq = nextSeq++;
    criticalQ.push_back({seq, std::string(line)});
    if (seqOut) *seqOut = seq;
    return FW_LOG_OK;
  }

  bool normalPost(const char *line) override {
    if ((int)normalQ.size() >= normalCap) { ++droppedNormal_; return false; }
    normalQ.push_back(std::string(line));
    return true;
  }

  // 处理一条：关键优先。holdDelivery 时把条目取出但不发布完成。
  bool pumpOne() override {
    if (!criticalQ.empty()) {
      const auto e = criticalQ.front();
      criticalQ.pop_front();
      if (holdDelivery) return true;      // 取出但未 println/flush
      return deliver(e.second, e.first);
    }
    if (!normalQ.empty()) {
      const std::string s = normalQ.front();
      normalQ.pop_front();
      if (holdDelivery) return true;
      return deliver(s, 0);
    }
    return false;
  }

  bool deliver(const std::string &line, uint32_t seq) {
    written.push_back(line);
    if (seq != 0) {
      flushedAfter.push_back(line);
      if (seq > lastDone) lastDone = seq;
    }
    return true;
  }

  bool seqDone(uint32_t seq) override { return seq != 0 && lastDone >= seq; }

  uint32_t nowMs() override { return clockMs; }
  void sleepMs(uint32_t ms) override { clockMs += (ms ? ms : sleepAdvanceMs); }

  uint32_t droppedNormal() override { return droppedNormal_; }
  uint32_t droppedCritical() override { return droppedCritical_; }
  uint32_t fmtErrors() override { return fmtErrors_; }

  bool nvsSavePeer(const uint8_t *peer6) override {
    ++nvsWrites;
    if (!nvsResult) return false;
    for (int i = 0; i < 6; ++i) savedPeer[i] = peer6[i];
    return true;
  }

  bool serialRead(int *out) override {
    if (input.empty()) return false;
    *out = (unsigned char)input.front();
    input.erase(input.begin());
    return true;
  }

  void restart() override { ++restartCount; }

  // ---- 测试辅助 ----
  void feed(const std::string &cmd) { input += cmd + "\n"; }
  void runQueue() { while (pumpOne()) {} }
  bool wrote(const std::string &s) const {
    for (const auto &w : written) if (w == s) return true;
    return false;
  }
  int countWrote(const std::string &s) const {
    int n = 0;
    for (const auto &w : written) if (w == s) ++n;
    return n;
  }
};
