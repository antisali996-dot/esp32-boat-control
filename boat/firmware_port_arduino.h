#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <esp_mac.h>

#include "firmware_port.h"
#include "run_log.h"
#include "pair_mac.h"

// 生产环境绑定：把 run_log.h 的逻辑接到真实的 FreeRTOS 队列、Serial、millis、NVS。
// 测试环境不使用本文件，而是绑定 tests/ 里的桩。

struct ArduinoFirmwarePort : public FirmwarePort {
  QueueHandle_t normalQueue = nullptr;
  QueueHandle_t criticalQueue = nullptr;
  TaskHandle_t task = nullptr;

  portMUX_TYPE doneMux = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t lastDone = 0;
  volatile uint32_t nextSeq = 1;

  volatile uint32_t droppedNormal_ = 0;
  volatile uint32_t droppedCritical_ = 0;
  volatile uint32_t fmtErrors_ = 0;

  bool begin() {
    normalQueue = xQueueCreate(RUN_LOG_QUEUE_DEPTH, sizeof(RunLogEntry));
    criticalQueue = xQueueCreate(RUN_LOG_CRITICAL_DEPTH, sizeof(RunLogEntry));
    if (!normalQueue || !criticalQueue) {
      if (normalQueue) { vQueueDelete(normalQueue); normalQueue = nullptr; }
      if (criticalQueue) { vQueueDelete(criticalQueue); criticalQueue = nullptr; }
      return false;
    }
    if (xTaskCreate(taskEntry, "runlog", 3072, this, 1, &task) != pdPASS) {
      vQueueDelete(normalQueue); normalQueue = nullptr;
      vQueueDelete(criticalQueue); criticalQueue = nullptr;
      task = nullptr;
      return false;
    }
    return true;
  }

  static void taskEntry(void *arg) {
    ArduinoFirmwarePort *self = static_cast<ArduinoFirmwarePort *>(arg);
    for (;;) self->pumpOne();
  }

  // ---- FirmwarePort ----
  FwLogPost criticalPost(const char *line, uint32_t *seqOut) override {
    if (seqOut) *seqOut = 0;
    if (!criticalQueue) { ++droppedCritical_; return FW_LOG_DISABLED; }
    RunLogEntry e;
    strncpy(e.text, line, sizeof(e.text) - 1);
    e.text[sizeof(e.text) - 1] = 0;
    portENTER_CRITICAL(&doneMux);
    e.seq = nextSeq++;
    portEXIT_CRITICAL(&doneMux);
    if (xQueueSend(criticalQueue, &e, 0) != pdTRUE) {
      ++droppedCritical_;
      return FW_LOG_QUEUE_FULL;
    }
    if (seqOut) *seqOut = e.seq;
    return FW_LOG_OK;
  }

  bool normalPost(const char *line) override {
    if (!normalQueue) return false;
    RunLogEntry e;
    strncpy(e.text, line, sizeof(e.text) - 1);
    e.text[sizeof(e.text) - 1] = 0;
    e.seq = 0;
    if (xQueueSend(normalQueue, &e, 0) != pdTRUE) {
      ++droppedNormal_;
      return false;
    }
    return true;
  }

  bool pumpOne() override {
    RunLogEntry e;
    bool critical = false;
    if (criticalQueue && xQueueReceive(criticalQueue, &e, 0) == pdTRUE) critical = true;
    else if (!normalQueue || xQueueReceive(normalQueue, &e, pdMS_TO_TICKS(20)) != pdTRUE) return false;

    Serial.println(e.text);
    if (critical) {
      Serial.flush();
      portENTER_CRITICAL(&doneMux);
      if (e.seq > lastDone) lastDone = e.seq;
      portEXIT_CRITICAL(&doneMux);
    }
    return true;
  }

  bool seqDone(uint32_t seq) override {
    portENTER_CRITICAL(&doneMux);
    const uint32_t d = lastDone;
    portEXIT_CRITICAL(&doneMux);
    return d >= seq;
  }

  uint32_t nowMs() override { return millis(); }
  void sleepMs(uint32_t ms) override { delay(ms); }

  uint32_t droppedNormal() override { return droppedNormal_; }
  uint32_t droppedCritical() override { return droppedCritical_; }
  uint32_t fmtErrors() override { return fmtErrors_; }

  bool nvsSavePeer(const uint8_t *peer6) override {
    Preferences prefs;
    bool ok = prefs.begin("jay-pair", false);
    if (ok) { ok = prefs.putBytes("peer", peer6, 6) == 6; prefs.end(); }
    return ok;
  }

  bool serialRead(int *out) override {
    if (Serial.available() <= 0) return false;
    *out = Serial.read();
    return true;
  }

  void restart() override { ESP.restart(); }
};

// 从 NVS 读取对端地址。返回是否读到有效值；空 NVS 或格式不对都不改动 peer。
// 放在生产绑定里，让 pairing.h 保持纯逻辑（电脑测试不需要 Arduino/Preferences）。
inline bool loadPair(uint8_t *peer) {
  Preferences prefs;
  if (!prefs.begin("jay-pair", false)) return false;
  uint8_t saved[6];
  bool loaded = false;
  if (prefs.getBytesLength("peer") == 6 && prefs.getBytes("peer", saved, 6) == 6 && validPeerMac(saved)) {
    memcpy(peer, saved, 6);
    loaded = true;
  }
  prefs.end();
  return loaded;
}

// esp_read_mac 的适配：返回 0 表示成功（esp_err_t 的 ESP_OK 就是 0）。
inline int readSelfMacRaw(uint8_t *out) {
  return (int)esp_read_mac(out, ESP_MAC_WIFI_STA);
}
