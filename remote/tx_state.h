#pragma once
#include <stdint.h>

// 调用者须持有 txMux；检查当前状态，而非循环早先取得的快照。
inline bool sendTimedOut(bool pending, uint32_t now, uint32_t sentAt, uint32_t timeout) {
  return pending && uint32_t(now - sentAt) >= timeout;
}
