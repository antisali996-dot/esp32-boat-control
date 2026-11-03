#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include "fw_role.h"
#include "self_mac.h"
#include "boot_info.h"
#include "steering.h"
#include "tx_state.h"
#include "tx_callback.h"
#include "pairing.h"
#include "run_log.h"
#include "firmware_port_arduino.h"

// 目标：已烧录的小船接收板，MAC 经串口读取确认。
uint8_t peerAddress[] = {0, 0, 0, 0, 0, 0};
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr int JOY_STEER = 32, BTN_UP = 25, BTN_DOWN = 26;
constexpr uint32_t SEND_INTERVAL_MS = 30;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
struct Payload { int32_t speedLimit; int32_t angle; };
static_assert(sizeof(Payload) == 8, "Boat expects two 32-bit integers");

bool sampleReady = false;
int rawSteer = 2048;
uint32_t lastSendMs = 0;
esp_err_t lastLocalError = ESP_OK;
SelfMac selfMac;
ArduinoFirmwarePort fwPort;
RunLog runLog;
PairingState pairing;

// ---- 发送状态：适配层 + 纯状态机 ----------------------------------------
// 适配层的职责：把 ESP-NOW 的“无名回调”变成“带在途标记的事件”，
// 并在临界区内一次性清掉标记；状态推进由主循环独占。
portMUX_TYPE txMux = portMUX_INITIALIZER_UNLOCKED;
TxStateMachine tx;
// 适配层与状态机分离：适配层负责“这次回调算不算数”，状态机负责“状态怎么走”。
// 二者共用 tx_callback.h 里的判定代码，电脑测试走的是同一条路径。
TxCallbackPort txPort;

// ESP-NOW 发送完成回调（WiFi 任务上下文）。
// 整段在临界区内执行：判定 + 吊销在途 + 记录事件必须原子完成。
void OnDataSent(const uint8_t *, esp_now_send_status_t status) {
  portENTER_CRITICAL(&txMux);
  txPort.onSendCallback(status == ESP_NOW_SEND_SUCCESS);
  portEXIT_CRITICAL(&txMux);
}

bool checkResult(const char *step, esp_err_t result) {
  Serial.printf("%s: %s\n", step, esp_err_to_name(result));
  return result == ESP_OK;
}

void addBoatPeer() {
  if (!validPeerMac(peerAddress)) return;
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, sizeof(peerAddress));
  peerInfo.channel = RADIO_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  const esp_err_t r = esp_now_add_peer(&peerInfo);
  const bool ok = (r == ESP_OK || r == ESP_ERR_ESPNOW_EXIST);
  if (!ok) Serial.printf("Add boat: %s\n", esp_err_to_name(r));
}

// 一次性初始化：WiFi 模式、信道、ESP-NOW、回调、对端。
bool startRadio() {
  if (!WiFi.mode(WIFI_STA)) { Serial.println("WiFi STA failed"); return false; }
  if (esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Set channel failed");
    return false;
  }
  uint8_t channel = 0;
  wifi_second_chan_t secondary;
  if (esp_wifi_get_channel(&channel, &secondary) != ESP_OK || channel != RADIO_CHANNEL) {
    Serial.printf("Channel mismatch: %u\n", channel);
    return false;
  }
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return false; }
  if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
    Serial.println("Send callback register failed");
    return false;
  }
  addBoatPeer();
  return true;
}

void setup() {
  Serial.begin(115200);
  // 先读自身地址：不依赖 WiFi 已启动，也不依赖是否已配对。
  selfMac.read(readSelfMacRaw);
  loadPair(peerAddress);
  pinMode(JOY_STEER, INPUT);
  // 小船自动定速：这两个按键当前不参与油门控制。
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  analogReadResolution(12);
  fwPort.begin();
  runLog.begin(&fwPort);
  delay(200);

  char selfText[16];
  selfMac.state.format(selfText, sizeof(selfText));
  // 启动一次性日志：直接输出，不经运行日志队列。
  Serial.printf("BOOT FW=%s\n", FW_ID);
  Serial.printf("BOOT VER=%s ROLE=%s MAC=%s RESET=%s(%d)\n", FW_VER, FW_ROLE_LABEL,
                selfText, resetReasonName(esp_reset_reason()), (int)esp_reset_reason());
  if (!selfMac.ok()) {
    Serial.printf("BOOT SELF_MAC_READ_FAILED esp=%s\n",
                  esp_err_to_name((esp_err_t)selfMac.status()));
  }
  Serial.printf("BOOT PEER=%02X:%02X:%02X:%02X:%02X:%02X CHANNEL=%u\n",
                peerAddress[0], peerAddress[1], peerAddress[2], peerAddress[3],
                peerAddress[4], peerAddress[5], RADIO_CHANNEL);
  pairInfo(FW_ROLE_LABEL, peerAddress, selfMac, runLog);

  if (!validPeerMac(peerAddress)) {
    Serial.println("PAIR REQUIRED / USB configuration needed");
    // 未配对：不发送，但 JAY INFO / JAY BIND 仍然可用。
    return;
  }
  if (startRadio()) {
    tx.onRadioUp();
    Serial.println("REMOTE READY / steering only / boat auto fixed RPM");
  } else {
    // 启动即失败：直接锁存故障，不做自动重试。
    Serial.println("RADIO START FAILED / TX FAULT / restart required");
    tx.state = TxStateMachine::FAULT;
  }
}

void loop() {
  servicePairing(FW_ROLE_LABEL, peerAddress, selfMac, runLog, fwPort, pairing);
  const uint32_t now = millis();

  static uint32_t lastSample = 0, sum = 0, lastReport = 0, lastStateReport = 0;
  static uint8_t samples = 0;
  static int lastTxState = -1;

  // 每毫秒采一笔，累计十笔平均。
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

  // ---- 吸收适配层事件（只在临界区内搬运，状态推进在主循环） ----
  uint32_t cbToken = 0;
  TxCallbackKind cbKind;
  portENTER_CRITICAL(&txMux);
  cbKind = txPort.take(&cbToken);
  portEXIT_CRITICAL(&txMux);
  if (cbKind == TX_CB_STALE) {
    tx.onSendCallback(0, false);                   // 迟到回调：只计数，不推进
  } else if (cbKind == TX_CB_SUCCESS || cbKind == TX_CB_FAILED) {
    tx.onSendCallback(cbToken, cbKind == TX_CB_SUCCESS);
  }

  // ---- 超时检查：一旦超时立刻锁存 FAULT，不自动重建 ----
  if (tx.tick(now)) {
    runLog.logLine("EVENT TX_FAULT=CALLBACK_TIMEOUT timeout_ms=%lu inflight=%lu",
            (unsigned long)TxStateMachine::SEND_TIMEOUT_MS,
            (unsigned long)txPort.inFlightToken);
    portENTER_CRITICAL(&txMux);
    // 超时后原子清空在途痕迹：之后的回调一律按迟到处理，不自动重建、不重新发送。
    txPort.disarm();
    txPort.pendingCallback = TX_CB_NONE;
    txPort.callbackToken = 0;
    portEXIT_CRITICAL(&txMux);
  }

  // ---- 发送 ----
  if (tx.canSend(now) && sampleReady && now - lastSendMs >= SEND_INTERVAL_MS) {
    const Payload packet = {100, angle}; // 第一字段仅兼容协议，小船忽略它。
    const uint32_t token = tx.beginSend(now);
    if (token != 0) {
      portENTER_CRITICAL(&txMux);
      txPort.arm(token);
      portEXIT_CRITICAL(&txMux);
      lastSendMs = now;
      lastLocalError = esp_now_send(peerAddress, reinterpret_cast<const uint8_t *>(&packet),
                                   sizeof(packet));
      if (lastLocalError != ESP_OK) {
        portENTER_CRITICAL(&txMux);
        txPort.disarm();
        portEXIT_CRITICAL(&txMux);
        tx.onLocalSendError();
      }
    }
  }

  // ---- 状态切换：只在变化时投递一次，不每轮重复 ----
  const int st = (int)tx.state;
  if (st != lastTxState) {
    lastTxState = st;
    runLog.logLine("EVENT TX_STATE=%s", tx.stateName());
  }

  // ---- 每秒运行日志：零等待投递，队列满则丢弃并计数 ----
  if (now - lastReport >= REPORT_PERIOD_MS) {
    lastReport = now;
    runLog.logLine("RUN state=%s adc=%d angle=%d ok=%lu peer_miss=%lu local_err=%lu cb_timeout=%lu stale=%lu last=%s",
            tx.stateName(), rawSteer, angle,
            (unsigned long)tx.delivered, (unsigned long)tx.sendFailCallbacks,
            (unsigned long)tx.localSendErrors, (unsigned long)tx.callbackTimeouts,
            (unsigned long)tx.staleCallbacks, esp_err_to_name(lastLocalError));
  }

  delay(1);
}
