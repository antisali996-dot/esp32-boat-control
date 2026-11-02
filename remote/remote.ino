#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include "steering.h"
#include "tx_state.h"
#include "pairing.h"

// 目标：已烧录的小船接收板，MAC 经串口读取确认。
uint8_t peerAddress[] = {0, 0, 0, 0, 0, 0};
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr int JOY_STEER = 32, BTN_UP = 25, BTN_DOWN = 26;
constexpr uint32_t SEND_INTERVAL_MS = 30, SEND_TIMEOUT_MS = 500;
struct Payload { int32_t speedLimit; int32_t angle; };
static_assert(sizeof(Payload) == 8, "Boat expects two 32-bit integers");

bool ready = false, sampleReady = false;
int rawSteer = 2048;
uint32_t localErrors = 0, lastSendMs = 0;
esp_err_t lastLocalError = ESP_OK;
portMUX_TYPE txMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool txPending = false;
volatile uint32_t delivered = 0, failed = 0;

// Arduino-ESP32 3.0.2 的发送完成回调；只记录结果，不在 WiFi 任务里打印。
void OnDataSent(const uint8_t *, esp_now_send_status_t status) {
  portENTER_CRITICAL(&txMux);
  if (status == ESP_NOW_SEND_SUCCESS) delivered = delivered + 1;
  else failed = failed + 1;
  txPending = false;
  portEXIT_CRITICAL(&txMux);
}

bool checkResult(const char *step, esp_err_t result) {
  Serial.printf("%s: %s\n", step, esp_err_to_name(result));
  return result == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  loadPair(peerAddress);
  pinMode(JOY_STEER, INPUT);
  // 当前小船自动定速 580 RPM：这两个按钮不参与油门控制。
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  analogReadResolution(12);
  delay(500);
  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("WiFi STA failed; sending disabled");
    return;
  }
  Serial.printf("Remote STA MAC: %s\n", WiFi.macAddress().c_str());
  pairInfo("REMOTE", peerAddress);
  if (!checkResult("Set channel", esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE))) return;
  uint8_t channel = 0;
  wifi_second_chan_t secondary;
  if (!checkResult("Read channel", esp_wifi_get_channel(&channel, &secondary))) return;
  Serial.printf("Actual channel: %u\n", channel);
  if (channel != RADIO_CHANNEL) return;
  if (!checkResult("ESP-NOW init", esp_now_init())) return;
  if (!checkResult("Send callback", esp_now_register_send_cb(OnDataSent))) return;
  if (!validPeerMac(peerAddress)) { Serial.println("PAIR REQUIRED / USB configuration needed"); return; }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, sizeof(peerAddress));
  peerInfo.channel = RADIO_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (!checkResult("Add boat", esp_now_add_peer(&peerInfo))) return;
  ready = true;
  Serial.println("REMOTE READY / steering only / boat auto 580 RPM");
}

void loop() {
  servicePairing("REMOTE", peerAddress);
  const uint32_t now = millis();
  static uint32_t lastSample = 0, sum = 0, lastReport = 0;
  static uint8_t samples = 0;
  // 每毫秒采一笔，累计十笔平均；按钮不会阻塞转向。
  if (now - lastSample >= 1) {
    lastSample = now;
    sum += analogRead(JOY_STEER);
    if (++samples == 10) {
      rawSteer = sum / 10;
      sum = 0;
      samples = 0;
      sampleReady = true;
    }
  }
  const int angle = steeringAngle(rawSteer);
  bool pending;
  uint32_t ok, fail;
  portENTER_CRITICAL(&txMux);
  pending = txPending;
  ok = delivered;
  fail = failed;
  portEXIT_CRITICAL(&txMux);
  // 异常情况下不无限等待回调，也不叠加尚未完成的发送。
  bool timedOut = false;
  portENTER_CRITICAL(&txMux);
  // 与回调互斥，按最新 pending 决定是否关闭发送。
  if (ready && sendTimedOut(txPending, now, lastSendMs, SEND_TIMEOUT_MS)) {
    ready = false;
    timedOut = true;
  }
  pending = txPending;
  portEXIT_CRITICAL(&txMux);
  if (timedOut) Serial.println("Send callback timeout; sending disabled, restart remote");
  if (ready && sampleReady && !pending && now - lastSendMs >= SEND_INTERVAL_MS) {
    const Payload packet = {100, angle}; // 第一字段仅兼容协议，小船忽略它。
    lastSendMs = now;
    portENTER_CRITICAL(&txMux);
    txPending = true;
    portEXIT_CRITICAL(&txMux);
    lastLocalError = esp_now_send(peerAddress, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (lastLocalError != ESP_OK) {
      ++localErrors;
      portENTER_CRITICAL(&txMux);
      txPending = false;
      portEXIT_CRITICAL(&txMux);
    }
  }
  if (now - lastReport >= 1000) {
    lastReport = now;
    Serial.printf("ready=%d adc=%d angle=%d delivered=%lu failed=%lu local_errors=%lu last=%s\n",
      ready, rawSteer, angle, (unsigned long)ok, (unsigned long)fail,
      (unsigned long)localErrors, esp_err_to_name(lastLocalError));
  }
  delay(1);
}
