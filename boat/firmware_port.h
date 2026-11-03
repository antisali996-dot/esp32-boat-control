#pragma once
#include <cstddef>
#include <cstdint>

// 固件依赖的平台接口（可注入）。
//
// 目的：让电脑测试执行的是固件真实的 logCritical / 完成确认 / 配对命令处理代码，
// 而不是在测试里另写一套 LogModel、SeqAckModel 去“模拟同样的行为”。
//
// 生产环境由 firmware_port.h 绑定 Arduino/FreeRTOS 实现；
// 测试环境在 tests/ 里绑定受控桩，并精确控制“投递成功/失败”“flush 何时完成”“超时”。
//
// 所有方法都是纯虚接口，不包含任何业务逻辑——业务逻辑只存在于 run_log.h 与 pairing.h。

// 一次关键行发送的完成状态。
enum FwLogPost {
  FW_LOG_OK = 0,
  FW_LOG_QUEUE_FULL = 1,
  FW_LOG_FMT_ERROR = 2,      // 格式化失败或会被截断
  FW_LOG_DISABLED = 3,       // 队列/任务未建立
};

// 串口 + 队列 + 时钟 + NVS + 重启的注入点。
struct FirmwarePort {
  virtual ~FirmwarePort() = default;

  // ---- 关键日志队列 ----
  // 投递一行到关键队列。成功时写入分配的序号。零等待：满则立刻返回 FW_LOG_QUEUE_FULL。
  virtual FwLogPost criticalPost(const char *line, uint32_t *seqOut) = 0;
  // 投递一行普通日志。零等待：满则立刻返回 false。
  virtual bool normalPost(const char *line) = 0;
  // 日志任务循环一次：取一条（关键优先），写入串口，关键行做 flush 并发布完成。
  // 返回是否处理了一条。生产实现里由独立任务循环调用。
  virtual bool pumpOne() = 0;
  // 查询“该序号是否已经 println + flush 完成”。
  virtual bool seqDone(uint32_t seq) = 0;

  // ---- 时间与等待 ----
  virtual uint32_t nowMs() = 0;
  // 让出 CPU 一小段时间（毫秒）。仅用于维护命令路径的等待循环。
  virtual void sleepMs(uint32_t ms) = 0;

  // ---- 计数（诊断用，生产环境直接暴露） ----
  virtual uint32_t droppedNormal() = 0;
  virtual uint32_t droppedCritical() = 0;
  virtual uint32_t fmtErrors() = 0;

  // ---- NVS ----
  virtual bool nvsSavePeer(const uint8_t *peer6) = 0;

  // ---- USB 串口输入 ----
  // 非阻塞读一个字节；有数据返回 true 并写入 out，否则返回 false。
  virtual bool serialRead(int *out) = 0;

  // ---- 重启 ----
  virtual void restart() = 0;
};
