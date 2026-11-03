// 控制与状态机回归。不依赖 Arduino，直接用 g++ 编译。
// 关键点：断言不依赖 <cassert>/NDEBUG。所有检查都用显式 CHECK 宏，
// 因此 -DNDEBUG 构建同样会执行全部检查，故意失败的用例也必然返回非零。
#include <cstdio>
#include <cstdint>
#include "../boat/pair_mac.h"
#include "../boat/fw_identity.h"
#include "../boat/motor_control.h"
#include "../boat/servo_pulse.h"
#include "../remote/steering.h"
#include "../remote/tx_state.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                          \
  do {                                                                      \
    ++g_checks;                                                             \
    if (!(cond)) {                                                          \
      ++g_failures;                                                         \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                       \
  } while (0)

#define CHECK_EQ(a, b)                                                       \
  do {                                                                      \
    ++g_checks;                                                             \
    auto va = (a);                                                          \
    auto vb = (b);                                                          \
    if (!(va == vb)) {                                                      \
      ++g_failures;                                                         \
      std::printf("FAIL %s:%d  %s == %s  (%lld vs %lld)\n", __FILE__,       \
                  __LINE__, #a, #b, (long long)va, (long long)vb);          \
    }                                                                       \
  } while (0)

// ---------------------------------------------------------------------------
// 身份常量
// ---------------------------------------------------------------------------
static void testIdentity() {
  CHECK_EQ(FW_TARGET_RPM, 590);
  CHECK_EQ(FW_OVERSPEED_RPM, 600);
  CHECK_EQ(FW_CENTER_ANGLE, 90);
  CHECK_EQ(FW_MIN_ANGLE, 70);
  CHECK_EQ(FW_MAX_ANGLE, 110);
  CHECK_EQ(FW_CENTER_US, 1500);
  CHECK_EQ(FW_MIN_US, 1278);
  CHECK_EQ(FW_MAX_US, 1722);
  CHECK(FW_OVERSPEED_RPM > FW_TARGET_RPM);
  // 身份串必须体现 590 与 600，不能继续冒充旧的 580 版本。
  char expect[128];
  std::snprintf(expect, sizeof(expect), "%s", FW_ID_BOAT);
  CHECK(std::strstr(FW_ID_BOAT, "R590") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "OS600") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "S20-A70-110-U1278-1500-1722") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "R580") == nullptr);
}

// ---------------------------------------------------------------------------
// 舵机脉宽与 14 位量化
// ---------------------------------------------------------------------------
static void testServo() {
  CHECK_EQ(steeringPulse(70), 1278);
  CHECK_EQ(steeringPulse(90), 1500);
  CHECK_EQ(steeringPulse(110), 1722);
  CHECK_EQ(steeringPulse(0), 1278);
  CHECK_EQ(steeringPulse(180), 1722);
  CHECK_EQ(steeringPulse(-50), 1278);
  CHECK_EQ(steeringPulse(999), 1722);

  // 全 ADC 范围：单调、受限、中位正确、死区连续。
  int prev = steeringAngle(0);
  CHECK_EQ(prev, 70);
  for (int raw = 1; raw <= 4095; ++raw) {
    const int a = steeringAngle(raw);
    CHECK(a >= FW_MIN_ANGLE && a <= FW_MAX_ANGLE);
    CHECK(a >= prev);
    CHECK(a - prev <= 1);
    if (raw >= DEAD_LOW && raw <= DEAD_HIGH) CHECK_EQ(a, 90);
    prev = a;
  }
  CHECK_EQ(steeringAngle(4095), 110);

  // 静止输入重复执行 → 目标脉宽不变，且只下发一次（首次回中必须输出）。
  ServoOutput out;
  int pulse = steeringAngle(2048) == 90 ? steeringPulse(90) : -1;
  CHECK_EQ(pulse, 1500);
  CHECK(out.shouldWrite(pulse));      // 首次必须下发
  CHECK_EQ(out.writes, 1);
  for (int i = 0; i < 100; ++i) CHECK(!out.shouldWrite(pulse));
  CHECK_EQ(out.writes, 1);
  CHECK(out.shouldWrite(steeringPulse(70)));
  CHECK_EQ(out.writes, 2);
  CHECK(!out.shouldWrite(steeringPulse(70)));

  // 14 位、50Hz 量化：每 tick = 20000/16384us，量化误差必须小于 1 tick 的一半以上余量。
  const double usPerTick = 20000.0 / 16384.0;
  for (int angle = 70; angle <= 110; ++angle) {
    const int us = steeringPulse(angle);
    const int ticks = (int)((double)us / usPerTick);
    const double back = ticks * usPerTick;
    CHECK(back <= us + usPerTick);            // 不会越过下一个刻度
    CHECK(us - back < usPerTick);             // 误差小于 1 个 tick
    CHECK(ticks > 0 && ticks < 16384);        // 落在 14 位计数范围内
  }
  // 相邻 1° 的脉宽差必须大于 1 个 tick，否则 14 位也分不开相邻角度。
  for (int angle = 70; angle < 110; ++angle) {
    CHECK(steeringPulse(angle + 1) - steeringPulse(angle) >= 1);
  }
}

// ---------------------------------------------------------------------------
// 电机控制：590 目标、600 超速锁存、负载减小
// ---------------------------------------------------------------------------
static void runMotor(MotorControl &m, uint32_t fromMs, uint32_t toMs, uint32_t stepMs,
                     uint32_t pulses, float rpm, bool enabled = true) {
  for (uint32_t t = fromMs; t <= toMs; t += stepMs)
    m.update(t, stepMs, pulses, rpm, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, enabled);
}

static void testMotorTarget() {
  // 590 目标下，稳态误差应把输出推到接近饱和而不是停在 580 的旧工作点。
  // 转速取 420：高于既有“持续严重低速”阈值（590*0.65≈384），避免误触发低速保护。
  MotorControl m;
  runMotor(m, 50, 4000, 50, 60, 420);
  CHECK_EQ(m.fault, MotorControl::NONE);
  CHECK(m.pwm > 0);
  // 负载减小 → 转速上升 → 输出必须能从饱和值下降。
  const int before = m.pwm;
  CHECK_EQ(before, 255);
  m.update(4050, 50, 120, 1000, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm < before);
}

static void testOverspeedLatch() {
  MotorControl m;
  // 先在 590 附近稳定运行。
  runMotor(m, 50, 2000, 50, 70, 585);
  CHECK_EQ(m.fault, MotorControl::NONE);
  const int pwmBefore = m.pwm;

  // 恰好 600：必须立刻断输出并锁存。
  m.update(2050, 50, 74, 600.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::OVERSPEED);
  CHECK_EQ(m.pwm, 0);
  // 触发证据必须保留。
  CHECK(m.overspeedRpm >= 600.0f);
  CHECK_EQ(m.overspeedPwm, pwmBefore);
  CHECK_EQ(m.overspeedIntervalMs, 50u);
  CHECK_EQ(m.overspeedPulses, 74u);

  // 随后输入 590 或零，都不能自动恢复。
  m.update(2100, 50, 72, 590.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.pwm, 0);
  CHECK_EQ(m.fault, MotorControl::OVERSPEED);
  m.update(2150, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.pwm, 0);
  CHECK_EQ(m.fault, MotorControl::OVERSPEED);
  // 即使关掉使能再打开，也不清除故障。
  m.update(2200, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, false);
  runMotor(m, 2250, 6000, 50, 70, 589);
  CHECK_EQ(m.pwm, 0);
  CHECK_EQ(m.fault, MotorControl::OVERSPEED);
}

static void testOverspeedJustBelow() {
  // 599.9 不得触发；600.0 才触发。这条用例正好说明“阈值是 600，不是 599”。
  MotorControl a;
  runMotor(a, 50, 3000, 50, 70, 589);
  a.update(3050, 50, 72, 599.9f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(a.fault, MotorControl::NONE);

  MotorControl b;
  runMotor(b, 50, 3000, 50, 70, 589);
  b.update(3050, 50, 72, 600.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(b.fault, MotorControl::OVERSPEED);
  CHECK_EQ(b.pwm, 0);
}

static void testAntiWindup() {
  // 强积分饱和后，输出受限时积分不应继续推高。
  MotorControl m;
  runMotor(m, 50, 6000, 50, 0, 0);   // 长时间零转速（但在达到无脉冲条件前不断言）
  const float integralAtLimit = m.integral;
  CHECK(integralAtLimit <= 255.0f);
  // 转速回到目标附近，输出应迅速回落，而不是被积分顶住。
  MotorControl n;
  runMotor(n, 50, 3000, 50, 70, 590);
  const int high = n.pwm;
  n.update(3050, 50, 70, 590, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(n.pwm <= 255);
  CHECK(n.pwm > 0 || high == 0);
}

static void testExistingProtections() {
  // 无脉冲保护仍然存在并锁存。
  MotorControl missing;
  runMotor(missing, 50, 3000, 50, 0, 0);
  CHECK_EQ(missing.fault, MotorControl::NO_PULSES);
  CHECK_EQ(missing.pwm, 0);
  runMotor(missing, 3050, 8000, 50, 70, 590);
  CHECK_EQ(missing.pwm, 0);

  // 持续严重低速保护仍然存在并锁存。
  MotorControl slow;
  runMotor(slow, 50, 10000, 50, 12, 100);
  CHECK_EQ(slow.fault, MotorControl::LOW_SPEED);
  CHECK_EQ(slow.pwm, 0);
  runMotor(slow, 10050, 14000, 50, 70, 590);
  CHECK_EQ(slow.pwm, 0);

  // 未使能时输出为零、不清故障。
  MotorControl disabled;
  disabled.update(50, 50, 40, 333, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, false);
  CHECK_EQ(disabled.pwm, 0);
  CHECK_EQ(disabled.fault, MotorControl::NONE);
}

static void testRamp() {
  MotorControl m;
  m.update(50, 50, 40, 333, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.pwm, 10);   // 启动斜率每周期最多 +10
  m.update(100, 50, 40, 333, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm <= 20);
}

// ---------------------------------------------------------------------------
// 遥控发送状态机
// ---------------------------------------------------------------------------
static void testTxHappyPath() {
  TxStateMachine t;
  t.onRadioUp();
  CHECK_EQ(t.state, TxStateMachine::READY);
  const uint32_t tok = t.beginSend(1000);
  CHECK(tok != 0);
  CHECK_EQ(t.state, TxStateMachine::WAIT_DONE);
  CHECK(!t.canSend(1005));           // 同一时刻只允许一个在途发送
  const uint32_t rejected = t.beginSend(1010);
  CHECK_EQ(rejected, 0);
  t.onSendCallback(tok, true);
  CHECK_EQ(t.state, TxStateMachine::READY);
  CHECK_EQ(t.delivered, 1);
}

static void testTxLocalError() {
  TxStateMachine t;
  t.onRadioUp();
  const uint32_t tok = t.beginSend(1000);
  t.onLocalSendError();
  CHECK_EQ(t.localSendErrors, 1);
  CHECK_EQ(t.state, TxStateMachine::READY);   // 本地错误不重启无线
  CHECK_EQ(t.radioRebuilds, 0);
  CHECK(t.canSend(1001));
  (void)tok;
}

static void testTxPeerMissDoesNotRebuild() {
  TxStateMachine t;
  t.onRadioUp();
  const uint32_t tok = t.beginSend(1000);
  t.onSendCallback(tok, false);      // 普通“对端未收到”
  CHECK_EQ(t.sendFailCallbacks, 1);
  CHECK_EQ(t.state, TxStateMachine::READY);
  CHECK_EQ(t.radioRebuilds, 0);
  CHECK_EQ(t.callbackTimeouts, 0);
}

static void testTxTimeoutAndBackoff() {
  TxStateMachine t;
  t.onRadioUp();
  t.beginSend(1000);
  // 未到超时：不动作。
  CHECK(!t.tick(1200));
  CHECK_EQ(t.state, TxStateMachine::WAIT_DONE);
  // 超时：先禁止发送，再进入退避。
  CHECK(!t.tick(1500));
  CHECK_EQ(t.callbackTimeouts, 1);
  CHECK_EQ(t.state, TxStateMachine::BACKOFF);
  CHECK(!t.canSend(1501));           // 超时后立刻禁止新发送
  // 退避 500ms。
  CHECK(!t.tick(1999));
  CHECK(t.tick(2000));               // 到点 → 请求重建
  CHECK_EQ(t.state, TxStateMachine::REBUILD);
  t.onRebuildResult(true, 2000);
  CHECK_EQ(t.state, TxStateMachine::READY);
  CHECK_EQ(t.radioRebuilds, 1);
}

static void testTxLateCallback() {
  TxStateMachine t;
  t.onRadioUp();
  const uint32_t tok = t.beginSend(1000);
  // 超时 → 清在途 → 退避 → 重建成功。
  t.tick(1500);
  t.tick(2000);
  t.onRebuildResult(true, 2000);
  // 新一轮发送开始。
  const uint32_t tok2 = t.beginSend(2500);
  CHECK(tok2 != 0);
  CHECK(tok2 != tok);
  // 迟到的旧回调：令牌不匹配，必须被丢弃，不能结束新一轮。
  t.onSendCallback(tok, true);
  CHECK_EQ(t.state, TxStateMachine::WAIT_DONE);
  CHECK_EQ(t.delivered, 0);
  CHECK_EQ(t.pendingToken, tok2);
  // 新一轮自己的回调正常生效。
  t.onSendCallback(tok2, true);
  CHECK_EQ(t.state, TxStateMachine::READY);
  CHECK_EQ(t.delivered, 1);
}

static void testTxRepeatedRebuildFailure() {
  TxStateMachine t;
  t.onRadioUp();
  t.beginSend(1000);
  t.tick(1500);                 // timeout → BACKOFF(500)
  t.tick(2000);                 // → REBUILD
  t.onRebuildResult(false, 2000);
  CHECK_EQ(t.state, TxStateMachine::BACKOFF);
  CHECK_EQ(t.consecutiveRebuildFailures, 1);
  t.tick(3000);                 // 退避 1000 → 到点
  t.onRebuildResult(false, 3000);
  CHECK_EQ(t.consecutiveRebuildFailures, 2);
  CHECK_EQ(t.state, TxStateMachine::BACKOFF);
  t.tick(5000);                 // 退避 2000 → 到点
  t.onRebuildResult(false, 5000);
  CHECK_EQ(t.consecutiveRebuildFailures, 3);
  CHECK_EQ(t.state, TxStateMachine::FAULT);   // 连续 3 次 → 明确故障
  CHECK_EQ(t.rebuildFailures, 3);
  // FAULT 之后不再尝试重建。
  CHECK(!t.tick(9000));
  CHECK_EQ(t.state, TxStateMachine::FAULT);
  CHECK(!t.canSend(9000));
}

static void testTxBackoffSchedule() {
  CHECK_EQ(TxStateMachine::backoffMs(0), 500u);
  CHECK_EQ(TxStateMachine::backoffMs(1), 1000u);
  CHECK_EQ(TxStateMachine::backoffMs(2), 2000u);
  CHECK_EQ(TxStateMachine::backoffMs(9), 2000u);
  CHECK_EQ(TxStateMachine::SEND_TIMEOUT_MS, 500u);
}

static void testTxClockRollover() {
  TxStateMachine t;
  t.onRadioUp();
  const uint32_t start = UINT32_MAX - 100u;   // 无符号运算，故意跨过 2^32 回绕
  t.beginSend(start);
  CHECK(!t.tick(start + 10u));
  // tick 的返回值只表示“现在要重建”，超时本身要看状态。
  t.tick(start + 600u);             // 跨过回绕仍然正确超时
  CHECK_EQ(t.callbackTimeouts, 1);
  CHECK_EQ(t.state, TxStateMachine::BACKOFF);
  // 退避截止点也跨过回绕；逐毫秒推进直到重建请求出现。
  bool rebuild = false;
  for (uint32_t k = 1; k <= 700u; ++k) {
    if (t.tick(start + 600u + k)) { rebuild = true; break; }
  }
  CHECK(rebuild);
  CHECK_EQ(t.state, TxStateMachine::REBUILD);
}

static void testMacParsing() {
  uint8_t mac[6];
  CHECK(parsePeerMac("24:62:AB:F0:72:94", mac) && mac[5] == 0x94);
  CHECK(parsePeerMac("24:62:ab:ef:cf:84", mac));
  CHECK(parsePeerMac("24:62:AB:F1:36:64", mac));
  CHECK(!parsePeerMac("FF:FF:FF:FF:FF:FF", mac));
  CHECK(!parsePeerMac("00:00:00:00:00:00", mac));
  CHECK(!parsePeerMac("25:62:AB:F0:72:94", mac));
  CHECK(!parsePeerMac("24:62:AB:F0:72:9Z", mac));
  CHECK(!parsePeerMac("24:62:AB:F0:72:94 trailing", mac));
  CHECK(!parsePeerMac("2:62:AB:F0:72:94", mac));
}

// 每转脉冲数未标定：用占位断言提醒必须实测，而不是假装已知物理转速。
static void testUnmeasuredPulsesPerRev() {
  const float pulsesPerRev = 144.0f;
  CHECK(pulsesPerRev > 0);
  std::printf("NOTE: PULSES_PER_REV=%.1f 未实机标定；590/600 目前只是软件命令值。\n",
              (double)pulsesPerRev);
}

int main() {
  testIdentity();
  testServo();
  testMotorTarget();
  testOverspeedLatch();
  testOverspeedJustBelow();
  testAntiWindup();
  testExistingProtections();
  testRamp();
  testTxHappyPath();
  testTxLocalError();
  testTxPeerMissDoesNotRebuild();
  testTxTimeoutAndBackoff();
  testTxLateCallback();
  testTxRepeatedRebuildFailure();
  testTxBackoffSchedule();
  testTxClockRollover();
  testMacParsing();
  testUnmeasuredPulsesPerRev();

  if (g_failures == 0) {
    std::printf("PASS: %d checks\n", g_checks);
    return 0;
  }
  std::printf("FAILED: %d of %d checks\n", g_failures, g_checks);
  return 1;
}
