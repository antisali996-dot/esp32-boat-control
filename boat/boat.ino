#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "fw_role.h"
#include "motor_control.h"
#include "servo_pulse.h"
#include "pairing.h"

// Arduino-ESP32 3.x; TS90A 必须使用适配电压，本程序不能防止过压发热。

// ---- 控制参数 -----------------------------------------------------------
constexpr int AIN1 = 16, AIN2 = 18, PWMA = 4;
constexpr int SERVO_PIN = 25, ENCODER_A = 34;
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr uint32_t LINK_TIMEOUT_MS = 1000;
constexpr uint32_t CONTROL_PERIOD_MS = 50;
constexpr uint32_t REPORT_PERIOD_MS = 1000;
constexpr uint32_t DISPLAY_PERIOD_MS = 500;
constexpr uint32_t DISPLAY_REINIT_PERIOD_MS = 10000;
// 每转脉冲数仍需实测标定，本包不擅自改动。
constexpr float PULSES_PER_REV = 144.0f;
constexpr int TARGET_RPM = FW_TARGET_RPM;
constexpr int OVERSPEED_RPM = FW_OVERSPEED_RPM;
// 用户要求：上电自动定速，不依赖遥控器油门或是否连接。
constexpr bool MOTOR_ENABLED = true;
constexpr uint8_t MOTOR_CHANNEL = 2;

// 显示任务优先级不高于主循环（loopTask=1）；主循环从不等待显示任务。
constexpr int DISPLAY_TASK_PRIORITY = 1;
constexpr uint32_t DISPLAY_TASK_STACK = 4096;

struct Payload { int32_t speedLimit; int32_t angle; };
static_assert(sizeof(Payload) == 8, "Remote packet must contain two 32-bit integers");

// ---- 共享状态 -----------------------------------------------------------
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseCount = 0, rxCount = 0, badPackets = 0, lastRxMs = 0;
volatile int requestedAngle = FW_CENTER_ANGLE, requestedSpeed = 0;
bool radioReady = false, servoReady = false, motorReady = false;
uint8_t REMOTE_MAC[6] = {0, 0, 0, 0, 0, 0};
Servo myservo;
ServoOutput servoOut;
float actualRPM = 0;
int currentPwm = 0;
MotorControl motor;

// ---- 显示快照 -----------------------------------------------------------
struct DisplaySnapshot {
  bool linked;
  int angle;
  int pulseUs;
  int pwm;
  int targetRpm;
  int simRpm;
  MotorControl::Fault fault;
};
// 长度 1 的队列：主循环非阻塞覆盖写入，显示任务只读，双方都不等待对方。
QueueHandle_t displayQueue = nullptr;
TaskHandle_t displayTaskHandle = nullptr;
bool displayEnabled = false;

// 显示任务写入的错误码，主循环只读取用于打印一次。
volatile int displayErrorCode = 0;      // 0 无 / 1 初始化失败 / 2 发送失败
volatile bool displayErrorReported = false;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 27, 26);
bool displayOnline = false;
uint8_t displayAddress = 0;

void IRAM_ATTR readEncoder() {
  portENTER_CRITICAL_ISR(&stateMux);
  pulseCount = pulseCount + 1;
  portEXIT_CRITICAL_ISR(&stateMux);
}

void OnMessageRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Payload packet;
  bool valid = validPeerMac(REMOTE_MAC) && info && memcmp(info->src_addr, REMOTE_MAC, 6) == 0 &&
               data && len == sizeof(packet);
  if (valid) {
    memcpy(&packet, data, sizeof(packet));
    valid = packet.speedLimit >= 0 && packet.speedLimit <= 100 &&
            packet.angle >= FW_MIN_ANGLE && packet.angle <= FW_MAX_ANGLE;
  }
  const uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  if (valid) {
    requestedAngle = packet.angle;
    requestedSpeed = packet.speedLimit;
    lastRxMs = now;
    rxCount = rxCount + 1;
  } else {
    badPackets = badPackets + 1;
  }
  portEXIT_CRITICAL(&stateMux);
}

bool checkResult(const char *step, esp_err_t result) {
  Serial.printf("%s: %s\n", step, esp_err_to_name(result));
  return result == ESP_OK;
}

// ---- 显示任务：Wire / U8g2 只在这里访问 --------------------------------
// 只探测两种常见 SSD1306 地址；ACK 仅证明设备应答，不证明屏幕像素正常。
bool displayResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void displayTask(void *) {
  uint32_t lastInit = 0;
  for (;;) {
    DisplaySnapshot snap;
    // 这里的阻塞只影响显示任务自己的节奏；主循环从不等待它。
    if (xQueueReceive(displayQueue, &snap, pdMS_TO_TICKS(DISPLAY_PERIOD_MS)) != pdTRUE) continue;

    const uint32_t now = millis();
    const bool wasOnline = displayOnline;

    // 掉线探测与恢复全部在显示任务内完成。
    if (!displayOnline || !displayResponds(displayAddress)) {
      displayOnline = false;
      for (uint8_t address : {uint8_t(0x3C), uint8_t(0x3D)}) {
        if (displayResponds(address)) { displayAddress = address; displayOnline = true; break; }
      }
    }
    if (!displayOnline) {
      if (wasOnline) {
        Serial.println("OLED OFFLINE");
        displayErrorCode = 2;   // 刷新/探测失败，由主循环报告一次；控制不受影响
      }
      continue;
    }

    // 即使两次探测之间 OLED 单独复位，也在最多约 10 秒后重发初始化。
    if (!wasOnline || uint32_t(now - lastInit) >= DISPLAY_REINIT_PERIOD_MS) {
      u8g2.setI2CAddress(displayAddress << 1);
      u8g2.initDisplay();
      u8g2.setPowerSave(0);
      u8g2.setFont(u8g2_font_6x12_tr);
      lastInit = now;
      Serial.printf("OLED INIT address=0x%02X\n", displayAddress);
    }

    u8g2.clearBuffer();
    u8g2.setCursor(0, 12); u8g2.print(snap.linked ? "RADIO OK" : "RADIO LOST");
    // 屏幕上的转速是模拟值，不参与控制。
    u8g2.setCursor(0, 28); u8g2.print("SIM RPM:"); u8g2.print(snap.simRpm);
    u8g2.setCursor(0, 44); u8g2.print("ANG:"); u8g2.print(snap.angle);
    u8g2.print(" "); u8g2.print(snap.pulseUs);
    u8g2.setCursor(0, 60);
    switch (snap.fault) {
      case MotorControl::OVERSPEED: u8g2.print("FAULT OVERSPEED"); break;
      case MotorControl::NO_PULSES: u8g2.print("FAULT NO PULSE"); break;
      case MotorControl::LOW_SPEED: u8g2.print("FAULT LOW SPEED"); break;
      default: u8g2.print("TARGET:"); u8g2.print(snap.targetRpm); break;
    }
    u8g2.sendBuffer();
    // sendBuffer() 无返回值；用一次总线探测确认本帧确实送达，失败留给下一轮恢复。
    if (!displayResponds(displayAddress)) {
      displayOnline = false;
      displayErrorCode = 2;
    }
  }
}

bool startDisplayTask() {
  displayQueue = xQueueCreate(1, sizeof(DisplaySnapshot));
  if (!displayQueue) return false;
  if (xTaskCreatePinnedToCore(displayTask, "display", DISPLAY_TASK_STACK, nullptr,
                              DISPLAY_TASK_PRIORITY, &displayTaskHandle, 0) != pdPASS) {
    vQueueDelete(displayQueue);
    displayQueue = nullptr;
    displayTaskHandle = nullptr;
    return false;
  }
  return true;
}

// 非阻塞覆盖写入：队列满时用新快照覆盖旧的，主循环绝不等待。
void publishDisplay(const DisplaySnapshot &snap) {
  if (!displayEnabled) return;
  if (xQueueOverwrite(displayQueue, &snap) != pdTRUE) displayEnabled = false;
}

const char *faultName(MotorControl::Fault f) {
  switch (f) {
    case MotorControl::NO_PULSES: return "NO_ENCODER_PULSES";
    case MotorControl::LOW_SPEED: return "SUSTAINED_LOW_SPEED";
    case MotorControl::OVERSPEED: return "OVERSPEED";
    default: return "NONE";
  }
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

void setup() {
  // 在任何初始化之前保持动力 PWM 关闭。
  pinMode(PWMA, OUTPUT);
  digitalWrite(PWMA, LOW);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  Serial.begin(115200);
  // 控制路径不等串口：日志出口不阻塞时直接丢弃。启动横幅只输出一次。
  loadPair(REMOTE_MAC);
  delay(200);
  Serial.printf("BOOT FW=%s\n", FW_ID);
  Serial.printf("BOOT VER=%s ROLE=%s MAC=%s RESET=%s(%d)\n", FW_VER, FW_ROLE_LABEL,
                WiFi.macAddress().c_str(), resetReasonName(esp_reset_reason()),
                (int)esp_reset_reason());
  Serial.printf("BOOT TARGET_RPM=%d OVERSPEED_RPM=%d STEER_DEG=%d ANGLE=%d..%d PULSE=%d..%dus\n",
                TARGET_RPM, OVERSPEED_RPM, FW_STEER_DEG, FW_MIN_ANGLE, FW_MAX_ANGLE,
                FW_MIN_US, FW_MAX_US);

  // 舵机库分配在 timer 0；动力用 LEDC 通道 2，映射到 timer 1，二者不同定时器。
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  // 必须在 attach 之前显式设 14 位：50Hz 下 20000us/16384 ≈ 1.22us 分辨率。
  myservo.setTimerWidth(14);
  myservo.attach(SERVO_PIN, 1000, 2000);
  servoReady = myservo.attached();

  if (MOTOR_ENABLED) {
    motorReady = ledcAttachChannel(PWMA, 20000, 8, MOTOR_CHANNEL);
    if (motorReady) ledcWrite(PWMA, 0);
  }

  pinMode(ENCODER_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, FALLING);

  bool wifiReady = WiFi.mode(WIFI_STA);
  if (wifiReady) {
    pairInfo(FW_ROLE_LABEL, REMOTE_MAC);
    bool channelOK = checkResult("Set channel", esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));
    if (channelOK && checkResult("ESP-NOW init", esp_now_init())) {
      radioReady = checkResult("Receive callback", esp_now_register_recv_cb(OnMessageRecv));
    }
  } else {
    Serial.println("BOOT WIFI_STA=FAIL");
  }
  uint8_t channel = 0;
  wifi_second_chan_t secondary;
  checkResult("Read channel", esp_wifi_get_channel(&channel, &secondary));
  Serial.printf("Actual channel: %u\n", channel);
  Serial.printf("Servo attached: %s  Motor PWM: %s\n", servoReady ? "YES" : "NO",
                motorReady ? "YES" : "NO");

  // 显示在独立任务里启动；失败只禁用显示，控制继续。
  Wire.begin(26, 27, 100000);
  Wire.setTimeOut(20);
  u8g2.setBusClock(100000);
  displayEnabled = startDisplayTask();
  if (!displayEnabled) {
    Serial.println("DISPLAY TASK CREATE FAILED / display disabled, control continues");
  }
  const DisplaySnapshot first = {false, FW_CENTER_ANGLE, SERVO_CENTER_US, 0, TARGET_RPM, 0,
                                 MotorControl::NONE};
  publishDisplay(first);
}

void loop() {
  servicePairing(FW_ROLE_LABEL, REMOTE_MAC);

  static uint32_t lastControl = millis(), lastReport = 0, lastDisplayPush = 0;
  static bool controlInitialized = false;
  static uint32_t maxControlGapMs = 0;
  static int lastLink = -1, lastFault = -1, lastDisplayEnabled = -1;

  uint32_t count, bad, receivedAt;
  int angle, speed;
  portENTER_CRITICAL(&stateMux);
  count = rxCount; bad = badPackets; receivedAt = lastRxMs;
  angle = requestedAngle; speed = requestedSpeed;
  portEXIT_CRITICAL(&stateMux);
  // 在取得接收快照之后采样，避免新包时间晚于 now 导致无符号下溢。
  const uint32_t now = millis();
  const bool linked = radioReady && count > 0 && uint32_t(now - receivedAt) < LINK_TIMEOUT_MS;
  const int safeAngle = linked ? constrain(angle, FW_MIN_ANGLE, FW_MAX_ANGLE) : FW_CENTER_ANGLE;
  const int targetRPM = TARGET_RPM; // 遥控数据仅控制转向，油门字段不参与调速。

  // 舵机：失联立即回中；只有目标脉宽变化才下发，首次回中一定下发。
  if (servoReady) {
    const int pulseUs = steeringPulse(safeAngle);
    if (servoOut.shouldWrite(pulseUs)) myservo.writeMicroseconds(pulseUs);
  }

  // ---- 控制周期：唯一决定动力输出的地方 ----
  const uint32_t elapsed = now - lastControl;
  if (!controlInitialized || elapsed >= CONTROL_PERIOD_MS) {
    portENTER_CRITICAL(&stateMux);
    const uint32_t pulses = pulseCount;
    pulseCount = 0;
    portEXIT_CRITICAL(&stateMux);
    if (controlInitialized && elapsed > maxControlGapMs) maxControlGapMs = elapsed;
    // 首个周期没有真实间隔，按标称周期计算，避免除零或算出荒谬转速。
    const uint32_t measured = controlInitialized ? elapsed : CONTROL_PERIOD_MS;
    actualRPM = pulses * 60000.0f / (PULSES_PER_REV * measured);
    lastControl = now;
    controlInitialized = true;
    motor.update(now, measured, pulses, actualRPM, (float)targetRPM, (float)OVERSPEED_RPM,
                 MOTOR_ENABLED && motorReady && servoReady);
    currentPwm = motor.pwm;
  }

  if (MOTOR_ENABLED && motorReady) {
    digitalWrite(AIN1, currentPwm > 0 ? HIGH : LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, currentPwm);
  }

  // ---- 状态边沿：每次变化只打印一次 ----
  const int linkNow = linked ? 1 : 0;
  if (lastLink != linkNow) {
    lastLink = linkNow;
    Serial.printf("EVENT LINK=%s count=%lu\n", linked ? "OK" : "LOST", (unsigned long)count);
  }
  if (lastFault != (int)motor.fault) {
    lastFault = (int)motor.fault;
    if (motor.fault == MotorControl::OVERSPEED) {
      // 超速按需求单独记录触发证据，只在锁存瞬间输出一次。
      Serial.printf("EVENT MOTOR_FAULT=OVERSPEED rpm=%.1f pwm=%d pulses=%lu interval=%lums target=%d threshold=%d\n",
                    motor.overspeedRpm, motor.overspeedPwm, (unsigned long)motor.overspeedPulses,
                    (unsigned long)motor.overspeedIntervalMs, targetRPM, OVERSPEED_RPM);
    } else {
      Serial.printf("EVENT MOTOR_FAULT=%s\n", faultName(motor.fault));
    }
  }
  if (lastDisplayEnabled != (displayEnabled ? 1 : 0)) {
    lastDisplayEnabled = displayEnabled ? 1 : 0;
    if (!displayEnabled) Serial.println("EVENT DISPLAY=DISABLED (control continues)");
  }
  if (displayErrorCode != 0 && !displayErrorReported) {
    displayErrorReported = true;
    Serial.printf("EVENT DISPLAY_ERROR=%d (control continues)\n", (int)displayErrorCode);
  }

  // ---- 周期报告：每秒一次；串口拥塞时只丢弃日志，不阻塞控制 ----
  if (now - lastReport >= REPORT_PERIOD_MS) {
    lastReport = now;
    const uint32_t age = count > 0 ? uint32_t(now - receivedAt) : 0;
    Serial.printf("RUN up=%lus rpm=%.1f pulses=%lu target=%d pwm=%d integral=%.1f maxgap=%lums\n",
                  (unsigned long)(now / 1000), actualRPM, (unsigned long)motor.lastPulses,
                  targetRPM, currentPwm, motor.integral, (unsigned long)maxControlGapMs);
    Serial.printf("RUN fault=%s rx=%lu bad=%lu lastrx=%lums age=%lums servo=%dus\n",
                  faultName(motor.fault), (unsigned long)count, (unsigned long)bad,
                  (unsigned long)receivedAt, (unsigned long)age, steeringPulse(safeAngle));
  }

  // ---- 显示快照：非阻塞覆盖写入 ----
  if (displayEnabled && now - lastDisplayPush >= DISPLAY_PERIOD_MS) {
    lastDisplayPush = now;
    DisplaySnapshot snap;
    snap.linked = linked;
    snap.angle = safeAngle;
    snap.pulseUs = steeringPulse(safeAngle);
    snap.pwm = currentPwm;
    snap.targetRpm = targetRPM;
    // 用户要求的模拟显示 540～600，不参与控制。
    snap.simRpm = random(540, 601);
    snap.fault = motor.fault;
    publishDisplay(snap);
  }

  delay(5);
}
