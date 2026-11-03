#include <Arduino.h>
#include <inttypes.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "fw_role.h"
#include "self_mac.h"
#include "boot_info.h"
#include "motor_control.h"
#include "servo_pulse.h"
#include "speed_window.h"
#include "pulse_count.h"
#include "fault_evidence.h"
#include "diagnostic_commands.h"
#include "pairing.h"
#include "run_log.h"
#include "firmware_port_arduino.h"

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
volatile uint32_t rxCount = 0, badPackets = 0, lastRxMs = 0;
// 累计脉冲：与窗口计数在同一临界区内递增，只在构造时为 0。
PulseCounter pulses;
// 本次启动的诊断标识（32 个十六进制字符），不写 NVS、不作安全凭据。
BootId bootId;
// 首次故障证据（只在首次故障时锁存，之后不覆盖）。
FaultEvidence evidence;
DiagHooks diagHooks;
// 最近一次有效采样快照（RUN 行与 JAY COUNT 用同一来源）。
PulseSnapshot lastSample;
bool haveSample = false;
volatile int requestedAngle = FW_CENTER_ANGLE, requestedSpeed = 0;
bool radioReady = false, servoReady = false, motorReady = false;
uint8_t REMOTE_MAC[6] = {0, 0, 0, 0, 0, 0};
SelfMac selfMac;
ArduinoFirmwarePort fwPort;
RunLog runLog;
PairingState pairing;
Servo myservo;
ServoOutput servoOut;
float actualRPM = 0;
int currentPwm = 0;
MotorControl motor;
uint32_t maxControlGapMs = 0;

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
  // 窗口计数与累计计数由 onEdge() 在同一临界区内一起递增。
  pulses.onEdge();
  portEXIT_CRITICAL_ISR(&stateMux);
}

// 读取计量快照：时间、窗口计数、total 必须来自同一个临界区。
PulseSnapshot snapshotPulses() {
  portENTER_CRITICAL(&stateMux);
  const uint32_t now = millis();
  const PulseSnapshot s = pulses.peek(now);
  portEXIT_CRITICAL(&stateMux);
  return s;
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
      if (wasOnline) displayErrorCode = 2;   // 由控制循环报告一次；本任务不直接打印
      continue;
    }

    if (!wasOnline || uint32_t(now - lastInit) >= DISPLAY_REINIT_PERIOD_MS) {
      u8g2.setI2CAddress(displayAddress << 1);
      u8g2.initDisplay();
      u8g2.setPowerSave(0);
      u8g2.setFont(u8g2_font_6x12_tr);
      lastInit = now;
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


// ---- 只读诊断命令：薄包装 ----------------------------------------------
// 格式化、投递与成功判定全部在 diagnostic_commands.h，
// 这里只负责“取真实快照 / 取锁存证据”，不保留第二套格式串或布尔转换。
static bool handleCount(RunLog &log) {
  const PulseSnapshot snap = snapshotPulses();   // 时间与 total 同源
  return diag::handleCount(log, bootId.hex, snap);
}

static bool handleFault(RunLog &log) {
  return diag::handleFault(log, evidence);       // 只读锁存证据，不改它
}

const char *faultName(MotorControl::Fault f) {
  switch (f) {
    case MotorControl::NO_PULSES: return "NO_ENCODER_PULSES";
    case MotorControl::LOW_SPEED: return "SUSTAINED_LOW_SPEED";
    case MotorControl::OVERSPEED: return "OVERSPEED";
    default: return "NONE";
  }
}


// esp_fill_random 的适配：用来生成 boot 标识。
static void fillRandom16(uint8_t *buf, size_t len) { esp_fill_random(buf, len); }

void setup() {
  // 在任何初始化之前保持动力 PWM 关闭。
  pinMode(PWMA, OUTPUT);
  digitalWrite(PWMA, LOW);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  Serial.begin(115200);
  // 本次启动的诊断标识：16 字节随机 → 32 个十六进制字符。不写 NVS、不作安全凭据。
  bootId.generate(fillRandom16);
  // 自身地址先读：不依赖 WiFi 已启动或是否已配对。
  selfMac.read(readSelfMacRaw);
  loadPair(REMOTE_MAC);
  fwPort.begin();
  runLog.begin(&fwPort);
  delay(200);

  char selfText[16];
  selfMac.state.format(selfText, sizeof(selfText));
  // 启动一次性横幅：直接输出，不进运行日志队列（setup 里串口不会被控制循环争用）。
  Serial.printf("BOOT FW=%s\n", FW_ID);
  Serial.printf("BOOT VER=%s ROLE=%s MAC=%s RESET=%s(%d)\n", FW_VER, FW_ROLE_LABEL,
                selfText, resetReasonName(esp_reset_reason()), (int)esp_reset_reason());
  if (!selfMac.ok()) {
    Serial.printf("BOOT SELF_MAC_READ_FAILED esp=%s\n",
                  esp_err_to_name((esp_err_t)selfMac.status()));
  }
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
    pairInfo(FW_ROLE_LABEL, REMOTE_MAC, selfMac, runLog);
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

  // 只读诊断钩子（遥控器不设置，避免伪造编码器数据）。
  diagHooks.handleCount = handleCount;
  diagHooks.handleFault = handleFault;

  displayEnabled = startDisplayTask();
  if (!displayEnabled) {
    Serial.println("DISPLAY TASK CREATE FAILED / display disabled, control continues");
  }
  const DisplaySnapshot first = {false, FW_CENTER_ANGLE, SERVO_CENTER_US, 0, TARGET_RPM, 0,
                                 MotorControl::NONE};
  publishDisplay(first);
}

void loop() {
  servicePairing(FW_ROLE_LABEL, REMOTE_MAC, selfMac, runLog, fwPort, pairing, diagHooks);

  static SpeedWindow speed;
  static uint32_t lastControl = 0, lastReport = 0, lastDisplayPush = 0;
  static bool controlInitialized = false;
  static int lastLink = -1, lastFault = -1, lastDisplayEnabled = -1;

  uint32_t count, bad, receivedAt;
  int angle, speedLimit;
  portENTER_CRITICAL(&stateMux);
  count = rxCount; bad = badPackets; receivedAt = lastRxMs;
  angle = requestedAngle; speedLimit = requestedSpeed;
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

  // ---- 测速窗口：时间/窗口计数/total 在同一临界区内取同一快照 ----
  if (!controlInitialized) {
    uint32_t t0 = 0;
    portENTER_CRITICAL(&stateMux);
    t0 = millis();                  // 临界区内重新采样，不用主循环早先的快照
    // 与测试共用的基准函数：只清 pulses.windowPulses，绝不清 totalPulses。
    lastSample = beginBaseline(pulses, t0);
    speed.begin(t0);                // 与清零同一时刻建立的基准
    haveSample = true;
    portEXIT_CRITICAL(&stateMux);
    controlInitialized = true;

    // 首轮：不计算 RPM、不出力，因此不可能触发超速。
    actualRPM = 0.0f;
    motor.update(t0, CONTROL_PERIOD_MS, 0, 0.0f, (float)targetRPM, (float)OVERSPEED_RPM,
                 false);        // enabled=false ⇒ pwm 清 0，且不判定超速
    currentPwm = 0;
  } else if (speed.ready(now)) {    // 外部只作预检查，真正判定在临界区内
    uint32_t tSwap = 0, interval = 0;
    uint32_t windowPulses = 0;
    uint64_t total = 0;
    bool windowTaken = false;
    portENTER_CRITICAL(&stateMux);
    tSwap = millis();               // 交换时刻重新采样：分母用这个时间
    interval = speed.elapsed(tSwap);
    if (interval >= CONTROL_PERIOD_MS) {
      // 采样时间、窗口计数、累计计数来自同一快照；只清窗口计数。
      const PulseSnapshot snap = pulses.takeWindow(tSwap, nullptr);
      windowPulses = snap.windowPulses;
      total = snap.totalPulses;
      lastSample = snap;
      haveSample = true;
      speed.begin(tSwap);           // 同一时刻建立下一窗口基准
      windowTaken = true;
    }
    portEXIT_CRITICAL(&stateMux);

    // 临界区外才做浮点控制、证据记录与日志。
    if (windowTaken) {
      if (interval > maxControlGapMs) maxControlGapMs = interval;
      actualRPM = SpeedWindow::rpmFromPulses(windowPulses, interval, PULSES_PER_REV);
      lastControl = tSwap;

      // 在 motor.update 之前构造“本次记录”，pwm/integral 用故障判断之前的值。
      const MeasureRecord cur = evidence.beginWindow(
          tSwap, actualRPM, currentPwm, windowPulses, interval, motor.integral);
      const MotorControl::Fault faultBefore = motor.fault;
      motor.update(tSwap, interval, windowPulses, actualRPM, (float)targetRPM,
                   (float)OVERSPEED_RPM, MOTOR_ENABLED && motorReady && servoReady);
      // 只有首次从 NONE 变成非 NONE 才锁存；证据用故障前的值。
      evidence.afterUpdate(faultBefore, motor.fault, cur);
      currentPwm = motor.pwm;
    }
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
    runLog.logLine("EVENT LINK=%s count=%lu", linked ? "OK" : "LOST", (unsigned long)count);
  }
  if (lastFault != (int)motor.fault) {
    lastFault = (int)motor.fault;
    if (motor.fault == MotorControl::OVERSPEED) {
      // 故障证据同时保存在 motor 的锁存快照里，不只依赖这条可能被丢弃的日志。
      runLog.logLine("EVENT MOTOR_FAULT=OVERSPEED rpm=%.1f pwm=%d pulses=%lu interval=%lums target=%d threshold=%d",
              motor.overspeedRpm, motor.overspeedPwm, (unsigned long)motor.overspeedPulses,
              (unsigned long)motor.overspeedIntervalMs, targetRPM, OVERSPEED_RPM);
    } else {
      runLog.logLine("EVENT MOTOR_FAULT=%s", faultName(motor.fault));
    }
  }
  if (lastDisplayEnabled != (displayEnabled ? 1 : 0)) {
    lastDisplayEnabled = displayEnabled ? 1 : 0;
    if (!displayEnabled) runLog.logLine("EVENT DISPLAY=DISABLED (control continues)");
  }
  if (displayErrorCode != 0 && !displayErrorReported) {
    displayErrorReported = true;
    runLog.logLine("EVENT DISPLAY_ERROR=%d (control continues)", (int)displayErrorCode);
  }

  // ---- 周期报告：每秒一次；零等待投递 ----
  if (now - lastReport >= REPORT_PERIOD_MS) {
    lastReport = now;
    const uint32_t age = count > 0 ? uint32_t(now - receivedAt) : 0;
    runLog.logLine("RUN up=%lus rpm=%.1f pulses=%lu target=%d pwm=%d integral=%.1f maxgap=%lums dropped=%lu",
            (unsigned long)(now / 1000), actualRPM, (unsigned long)motor.lastPulses,
            targetRPM, currentPwm, motor.integral, (unsigned long)maxControlGapMs,
            (unsigned long)fwPort.droppedNormal());
    runLog.logLine("RUN fault=%s rx=%lu bad=%lu lastrx=%lums age=%lums servo=%dus speed=%d",
            faultName(motor.fault), (unsigned long)count, (unsigned long)bad,
            (unsigned long)receivedAt, (unsigned long)age, steeringPulse(safeAngle), speedLimit);
    // 采样时间与 total 必须来自同一有效快照；首个有效窗口前用初始基准快照。
    runLog.logLine("RUN boot=%s sample_ms=%lu total=%" PRIu64,
                   bootId.hex, (unsigned long)lastSample.sampleMs, lastSample.totalPulses);
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
