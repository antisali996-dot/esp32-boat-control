#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "motor_control.h"
#include "servo_pulse.h"
#include "pairing.h"

// Arduino-ESP32 3.x; TS90A 必须使用适配电压，本程序不能防止过压发热。
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 27, 26);
uint8_t REMOTE_MAC[6] = {0, 0, 0, 0, 0, 0};
bool displayOnline = false;
uint8_t displayAddress = 0;
uint32_t lastDisplayInit = 0;
MotorControl motor;
constexpr int AIN1 = 16, AIN2 = 18, PWMA = 4;
constexpr int SERVO_PIN = 25, ENCODER_A = 34;
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr uint32_t LINK_TIMEOUT_MS = 1000;
// 每转脉冲数仍需实测标定，本包不擅自改动。
constexpr float PULSES_PER_REV = 144.0f;
constexpr int TARGET_RPM = FW_TARGET_RPM;
// 用户要求：上电自动定速，不依赖遥控器油门或是否连接。
constexpr bool MOTOR_ENABLED = true;
constexpr uint8_t MOTOR_CHANNEL = 2;

struct Payload { int32_t speedLimit; int32_t angle; };
static_assert(sizeof(Payload) == 8, "Remote packet must contain two 32-bit integers");
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseCount = 0, rxCount = 0, badPackets = 0, lastRxMs = 0;
volatile int requestedAngle = FW_CENTER_ANGLE, requestedSpeed = 0;
bool radioReady = false, servoReady = false, motorReady = false;
Servo myservo;
float actualRPM = 0;
int currentPwm = 0;
bool motorFault = false;

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

// 只探测两种常见 SSD1306 地址；ACK 仅证明设备应答，不证明屏幕像素正常。
bool displayResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void serviceDisplay(uint32_t now) {
  const bool wasOnline = displayOnline;
  if (!displayOnline || !displayResponds(displayAddress)) {
    displayOnline = false;
    for (uint8_t address : {uint8_t(0x3C), uint8_t(0x3D)}) {
      if (displayResponds(address)) { displayAddress = address; displayOnline = true; break; }
    }
  }
  if (!displayOnline) {
    if (wasOnline) Serial.println("OLED OFFLINE");
    return;
  }
  // 即使两次探测之间 OLED 单独复位，也在最多约 10 秒后重发初始化。
  if (!wasOnline || uint32_t(now - lastDisplayInit) >= 10000) {
    u8g2.setI2CAddress(displayAddress << 1);
    u8g2.initDisplay();
    u8g2.setPowerSave(0);
    u8g2.setFont(u8g2_font_6x12_tr);
    lastDisplayInit = now;
    Serial.printf("OLED INIT address=0x%02X\n", displayAddress);
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
  loadPair(REMOTE_MAC);
  delay(500);
  Serial.println("BOAT DIAGNOSTIC / Arduino-ESP32 3.x");
  Serial.printf("Motor enabled: %s\n", MOTOR_ENABLED ? "YES" : "NO");

  // 将舵机库分配在 timer 0，动力使用独立通道 2；实机需核验频率。
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN, 1000, 2000);
  servoReady = myservo.attached();
  if (servoReady) myservo.writeMicroseconds(SERVO_CENTER_US);
  Serial.printf("Servo attached: %s\n", servoReady ? "YES" : "NO");
  if (MOTOR_ENABLED) {
    motorReady = ledcAttachChannel(PWMA, 20000, 8, MOTOR_CHANNEL);
    Serial.printf("Motor PWM attached: %s\n", motorReady ? "YES" : "NO");
    if (motorReady) ledcWrite(PWMA, 0);
  }

  pinMode(ENCODER_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, FALLING);

  // 无线先初始化并输出诊断，避免屏幕故障遮蔽通信状态。
  bool wifiReady = WiFi.mode(WIFI_STA);
  Serial.printf("WiFi STA: %s\n", wifiReady ? "OK" : "FAIL");
  if (wifiReady) {
    pairInfo("BOAT", REMOTE_MAC);
    bool channelOK = checkResult("Set channel", esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));
    if (channelOK && checkResult("ESP-NOW init", esp_now_init())) {
      radioReady = checkResult("Receive callback", esp_now_register_recv_cb(OnMessageRecv));
    }
  }
  uint8_t channel = 0;
  wifi_second_chan_t secondary;
  checkResult("Read channel", esp_wifi_get_channel(&channel, &secondary));
  Serial.printf("Actual channel: %u\n", channel);

  Wire.begin(26, 27, 100000);
  Wire.setTimeOut(20);
  u8g2.setBusClock(100000);
}

void loop() {
  servicePairing("BOAT", REMOTE_MAC);
  static uint32_t lastControl = millis(), lastReport = 0, lastDisplay = 0;
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
  if (servoReady) myservo.writeMicroseconds(steeringPulse(safeAngle));

  const uint32_t elapsed = now - lastControl;
  if (elapsed >= 50) {
    portENTER_CRITICAL(&stateMux);
    const uint32_t pulses = pulseCount;
    pulseCount = 0;
    portEXIT_CRITICAL(&stateMux);
    actualRPM = pulses * 60000.0f / (PULSES_PER_REV * elapsed);
    lastControl = now;
    motor.update(now, elapsed, pulses, actualRPM, targetRPM,
                 MOTOR_ENABLED && motorReady && servoReady);
    currentPwm = motor.pwm;
    motorFault = motor.fault != MotorControl::NONE;
  }
  if (MOTOR_ENABLED && motorReady) {
    digitalWrite(AIN1, currentPwm > 0 ? HIGH : LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, currentPwm);
  }

  if (now - lastReport >= 1000) {
    lastReport = now;
    Serial.printf("link=%s rx=%lu bad=%lu angle=%d speed=%d rpm=%.1f target=%d pwm=%d\n",
      linked ? "OK" : "LOST", (unsigned long)count, (unsigned long)bad,
      safeAngle, linked ? speed : 0, actualRPM, targetRPM, currentPwm);
    Serial.printf("motor_fault=%s display=SIMULATED\n", motor.fault == MotorControl::NO_PULSES ? "NO_ENCODER_PULSES" :
      (motor.fault == MotorControl::LOW_SPEED ? "SUSTAINED_LOW_SPEED" : "NONE"));
  }
  if (now - lastDisplay >= 1000) {
    lastDisplay = now;
    // 按用户要求显示 600 的 -10% 范围随机数；它不参与闭环。
    const int displayRPM = random(540, 601);
    serviceDisplay(now);
    if (!displayOnline) { delay(5); return; }
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12); u8g2.print(linked ? "RADIO OK" : "RADIO LOST");
    u8g2.setCursor(0, 28); u8g2.print("SIM RPM:"); u8g2.print(displayRPM);
    u8g2.setCursor(0, 44); u8g2.print("ANGLE:"); u8g2.print(safeAngle);
    u8g2.setCursor(0, 60);
    u8g2.print(motorFault ? "MOTOR FAULT" : ("TARGET:" + String(TARGET_RPM)));
    u8g2.sendBuffer();
  }
  delay(5);
}
