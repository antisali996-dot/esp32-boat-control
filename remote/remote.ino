#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include "fw_role.h"
#include "steering.h"
#include "tx_state.h"
#include "pairing.h"

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
bool peerAdded = false;

// 回调只在临界区内记录事件，不直接推动恢复流程；全部推进都在 loop() 里做。
portMUX_TYPE txMux = portMUX_INITIALIZER_UNLOCKED;
TxStateMachine tx;
// 在途发送的令牌（0 表示无）。回调必须携带相同令牌才算有效，迟到回调一律丢弃。
volatile uint32_t inFlightToken = 0;
volatile int32_t pendingCallback = -1;  // -1 无 / 0 失败 / 1 成功
volatile uint32_t callbackToken = 0;

void OnDataSent(const uint8_t *, esp_now_send_status_t status) {
  portENTER_CRITICAL(&txMux);
  const uint32_t token = inFlightToken;
  // 没有在途发送，或令牌已被清掉：这是迟到回调，直接丢弃。
  if (token != 0) {
    pendingCallback = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
    callbackToken = token;
    // 立刻吊销令牌，确保同一个回调不会被处理两次。
    inFlightToken = 0;
  }
  portEXIT_CRITICAL(&txMux);
}

bool checkResult(const char *step, esp_err_t result) {
  Serial.printf("%s: %s\n", step, esp_err_to_name(result));
  return result == ESP_OK;
}

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

bool addBoatPeer() {
  if (!validPeerMac(peerAddress)) return false;
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, sizeof(peerAddress));
  peerInfo.channel = RADIO_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  // 已存在时 add 会返回 EXIST，视为可用。
  const esp_err_t r = esp_now_add_peer(&peerInfo);
  peerAdded = (r == ESP_OK || r == ESP_ERR_ESPNOW_EXIST);
  if (peerAdded) return true;
  Serial.printf("Add boat: %s\n", esp_err_to_name(r));
  return false;
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
  return addBoatPeer();
}

// 重建无线：先注销回调，再 deinit，再重新初始化。
// 顺序与依据：本地 libespnow.a 反汇编显示 mt_unregister_send_cb 把回调指针清零，
// mt_deinit 同样清零，而 mt_send_cb 调用前重新读取该指针。
// 残余窗口：deinit 不等待已进入核心的在途发送，这一点必须实测确认，见 README。
bool rebuildRadio() {
  esp_now_unregister_send_cb();
  esp_now_deinit();
  peerAdded = false;
  bool ok = startRadio();
  Serial.printf("RADIO REBUILD %s\n", ok ? "OK" : "FAILED");
  return ok;
}

void setup() {
  Serial.begin(115200);
  loadPair(peerAddress);
  pinMode(JOY_STEER, INPUT);
  // 小船自动定速：这两个按键当前不参与油门控制。
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  analogReadResolution(12);
  delay(200);

  Serial.printf("BOOT FW=%s\n", FW_ID);
  Serial.printf("BOOT VER=%s ROLE=%s MAC=%s RESET=%s(%d)\n", FW_VER, FW_ROLE_LABEL,
                WiFi.macAddress().c_str(), resetReasonName(esp_reset_reason()),
                (int)esp_reset_reason());
  Serial.printf("BOOT PEER=%02X:%02X:%02X:%02X:%02X:%02X CHANNEL=%u\n",
                peerAddress[0], peerAddress[1], peerAddress[2], peerAddress[3],
                peerAddress[4], peerAddress[5], RADIO_CHANNEL);

  pairInfo(FW_ROLE_LABEL, peerAddress);

  if (!validPeerMac(peerAddress)) {
    Serial.println("PAIR REQUIRED / USB configuration needed");
    tx.state = TxStateMachine::FAULT;   // 明确故障，不进入发送
    return;
  }
  if (startRadio()) {
    tx.onRadioUp();
    Serial.println("REMOTE READY / steering only / boat auto fixed RPM");
  } else {
    // 启动即失败：同样走退避重建，不无限快速重试。
    Serial.println("RADIO START FAILED / entering rebuild backoff");
    tx.state = TxStateMachine::REBUILD;
    tx.onRebuildResult(false, millis());
  }
}

void loop() {
  servicePairing(FW_ROLE_LABEL, peerAddress);
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

  // ---- 吸收回调事件（只在临界区内搬运数据） ----
  int32_t cb = -1;
  uint32_t cbToken = 0;
  portENTER_CRITICAL(&txMux);
  cb = pendingCallback;
  cbToken = callbackToken;
  pendingCallback = -1;
  portEXIT_CRITICAL(&txMux);
  if (cb >= 0) tx.onSendCallback(cbToken, cb == 1);

  // ---- 推进状态机 ----
  const bool wantRebuild = tx.tick(now);
  if (wantRebuild) {
    Serial.printf("EVENT TX_REBUILD attempt=%u backoff_ms=%lu\n",
                  (unsigned)tx.consecutiveRebuildFailures + 1,
                  (unsigned long)TxStateMachine::backoffMs(
                      tx.consecutiveRebuildFailures < 3 ? tx.consecutiveRebuildFailures : 2));
    const bool ok = rebuildRadio();
    tx.onRebuildResult(ok, millis());
  }

  // ---- 发送 ----
  if (tx.canSend(now) && sampleReady && now - lastSendMs >= SEND_INTERVAL_MS) {
    const Payload packet = {100, angle}; // 第一字段仅兼容协议，小船忽略它。
    const uint32_t token = tx.beginSend(now);
    if (token != 0) {
      portENTER_CRITICAL(&txMux);
      inFlightToken = token;
      portEXIT_CRITICAL(&txMux);
      lastSendMs = now;
      lastLocalError = esp_now_send(peerAddress, reinterpret_cast<const uint8_t *>(&packet),
                                   sizeof(packet));
      if (lastLocalError != ESP_OK) {
        // 本地错误：立刻吊销令牌，避免把随后到达的迟到回调算成这一次的结果。
        portENTER_CRITICAL(&txMux);
        inFlightToken = 0;
        portEXIT_CRITICAL(&txMux);
        tx.onLocalSendError();
      }
    }
  }

  // ---- 状态切换只打印一次 ----
  const int st = (int)tx.state;
  if (st != lastTxState) {
    lastTxState = st;
    Serial.printf("EVENT TX_STATE=%s\n", tx.stateName());
    if (tx.state == TxStateMachine::FAULT) {
      Serial.printf("EVENT TX_FAULT rebuild_failures=%u (restart required)\n",
                    (unsigned)tx.rebuildFailures);
    }
  }

  // ---- 每秒报告 ----
  if (now - lastReport >= REPORT_PERIOD_MS) {
    lastReport = now;
    Serial.printf("RUN state=%s adc=%d angle=%d ok=%lu peer_miss=%lu local_err=%lu cb_timeout=%lu rebuild=%lu/%lu last=%s\n",
                  tx.stateName(), rawSteer, angle,
                  (unsigned long)tx.delivered, (unsigned long)tx.sendFailCallbacks,
                  (unsigned long)tx.localSendErrors, (unsigned long)tx.callbackTimeouts,
                  (unsigned long)tx.radioRebuilds, (unsigned long)tx.rebuildFailures,
                  esp_err_to_name(lastLocalError));
  }

  delay(1);
}
