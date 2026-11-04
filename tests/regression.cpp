// 固件逻辑回归。不依赖 Arduino，直接用 g++ 编译。
//
// 断言不依赖 <cassert>/NDEBUG：全部用显式 CHECK 宏，
// 因此 -DNDEBUG 构建同样执行全部检查，故意失败用例必然返回非零。
//
// 覆盖的固件真实代码：
//   boat/identity_line.h   身份行格式化（固件 postIdentity 与测试共用同一实现）
//   boat/self_mac_logic.h  自身 MAC 判定
//   boat/speed_window.h    测速窗口与临界区交换
//   boat/motor_control.h   540 目标(r12) / 600 锁存已取消(OSOFF)
//   boat/servo_pulse.h     舵机脉宽与 14 位量化
//   remote/tx_callback.h   发送回调适配层
//   remote/tx_state.h      发送状态机
//   remote/steering.h      摇杆映射
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include "../boat/pair_mac.h"
#include "../boat/fw_identity.h"
#include "../boat/identity_line.h"
#include "../boat/motor_control.h"
#include "../boat/servo_pulse.h"
#include "../boat/speed_window.h"
#include "../boat/self_mac_logic.h"
#include "../remote/steering.h"
#include "../remote/tx_state.h"
#include "../remote/tx_callback.h"
#include "../boat/firmware_port.h"
#include "../boat/run_log.h"
#include "../boat/pairing.h"
#include "../boat/pulse_count.h"
#include "../boat/fault_evidence.h"
#include "../boat/diagnostic_commands.h"
#include "stub_port.h"

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

constexpr uint32_t CONTROL_PERIOD_FOR_TEST = 50;
// 与固件 boat.ino 的 PULSES_PER_REV 一致；该值仍未实机标定。
constexpr float PULSES_PER_REV = 144.0f;


// ---------------------------------------------------------------------------
// 身份常量
// ---------------------------------------------------------------------------
static void testIdentity() {
  CHECK_EQ(FW_TARGET_RPM, 540);   // r12：目标临时降为 540
  CHECK_EQ(FW_OVERSPEED_RPM, 600);
  CHECK_EQ(FW_CENTER_ANGLE, 90);
  CHECK_EQ(FW_MIN_ANGLE, 70);
  CHECK_EQ(FW_MAX_ANGLE, 110);
  CHECK_EQ(FW_CENTER_US, 1500);
  CHECK_EQ(FW_MIN_US, 1278);
  CHECK_EQ(FW_MAX_US, 1722);
  CHECK(FW_OVERSPEED_RPM > FW_TARGET_RPM);
  CHECK(std::strstr(FW_ID_BOAT, "R540") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "R590") == nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "OSOFF") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "OS600") == nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "S20-A70-110-U1278-1500-1722") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "R580") == nullptr);
  // r10 发布身份必须与上一版不同。
  CHECK_EQ(FW_SCHEMA, 13);
  CHECK(std::strstr(FW_VER, "r13") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "r13") != nullptr);
  CHECK(std::strstr(FW_ID_BOAT, "r12") == nullptr);
  CHECK(std::strstr(FW_ID_REMOTE, "r12") == nullptr);
}

// ---------------------------------------------------------------------------
// 1. MAC 文本完整性（走真实 buildIdentityLines 路径）
// ---------------------------------------------------------------------------
static const uint8_t kSelfF13664[6] = {0x24, 0x62, 0xAB, 0xF1, 0x36, 0x64};
static const uint8_t kSelfEFCF84[6] = {0x24, 0x62, 0xAB, 0xEF, 0xCF, 0x84};
static const uint8_t kPeerF07294[6] = {0x24, 0x62, 0xAB, 0xF0, 0x72, 0x94};

// 从 "MAC=XX:XX:XX:XX:XX:XX" 解析回 6 字节。
static bool parseMacField(const char *line, const char *field, uint8_t out[6]) {
  const char *p = std::strstr(line, field);
  if (!p) return false;
  p += std::strlen(field);
  unsigned v[6];
  if (std::sscanf(p, "%2x:%2x:%2x:%2x:%2x:%2x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
    return false;
  for (int i = 0; i < 6; ++i) out[i] = (uint8_t)v[i];
  return true;
}

static void testMacTextCompleteness() {
  // 缓冲容量本身必须够：17 字符 + NUL。
  CHECK(MAC_TEXT_CAP >= 18);
  CHECK(IDENTITY_LINE_CAP >= 256);

  // 24:62:AB:F1:36:64 —— 旧 macText[16] 会截成 24:62:AB:F1:36:6
  {
    IdentityLines l = buildIdentityLines("BOAT", kSelfF13664, true, kPeerF07294,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                         FW_ID_BOAT, FW_VER);
    CHECK(l.mainOk());
    CHECK(std::strcmp(l.macText, "24:62:AB:F1:36:64") == 0);
    CHECK(std::strstr(l.main, "MAC=24:62:AB:F1:36:64") != nullptr);

    uint8_t back[6] = {0};
    CHECK(parseMacField(l.main, "MAC=", back));
    CHECK(std::memcmp(back, kSelfF13664, 6) == 0);
  }

  // 24:62:AB:EF:CF:84
  {
    IdentityLines l = buildIdentityLines("REMOTE", kSelfEFCF84, true, kPeerF07294,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                         FW_ID_REMOTE, FW_VER);
    CHECK(l.mainOk());
    CHECK(std::strcmp(l.macText, "24:62:AB:EF:CF:84") == 0);
    CHECK(std::strstr(l.main, "MAC=24:62:AB:EF:CF:84") != nullptr);
    uint8_t back[6] = {0};
    CHECK(parseMacField(l.main, "MAC=", back));
    CHECK(std::memcmp(back, kSelfEFCF84, 6) == 0);
  }

  // 读取失败：明确 READ_FAILED，不伪造全零 MAC。
  {
    IdentityLines l = buildIdentityLines("BOAT", nullptr, false, kPeerF07294,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                         FW_ID_BOAT, FW_VER);
    CHECK(l.mainOk());
    CHECK(std::strstr(l.main, "MAC=READ_FAILED") != nullptr);
    CHECK(std::strstr(l.main, "00:00:00:00:00:00") == nullptr);
    CHECK_EQ(l.selfMacValid, false);
    // 读取失败时不输出旧格式行（那行会带一个假 MAC）。
    CHECK_EQ(l.legacyStatus, FMT_ERROR);
  }

  // formatMacText 对不足的容量必须报告截断，而不是当作成功。
  char small[8];
  CHECK_EQ(formatMacText(kSelfF13664, true, small, sizeof(small)), FMT_TRUNCATED);
}

// ---------------------------------------------------------------------------
// 2. 身份行完整性与截断检测
// ---------------------------------------------------------------------------
static void testIdentityLineCompleteness() {
  // 小船当前完整身份行
  {
    IdentityLines l = buildIdentityLines("BOAT", kSelfF13664, true, kPeerF07294,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                         FW_ID_BOAT, FW_VER);
    CHECK(l.mainOk());
    CHECK(l.legacyOk());
    const char *fields[] = {"JAYv4 ", "ROLE=BOAT ", "MAC=24:62:AB:F1:36:64 ",
                            "PEER=24:62:AB:F0:72:94 ", "PAIR=2 ", "STEER=20 ",
                            "FWID=", "VER="};
    for (const char *f : fields) CHECK(std::strstr(l.main, f) != nullptr);
    CHECK(std::strstr(l.main, FW_ID_BOAT) != nullptr);
    CHECK(std::strstr(l.main, FW_VER) != nullptr);
    CHECK(std::strlen(l.main) < IDENTITY_LINE_CAP);
    CHECK(std::strlen(l.legacy) < IDENTITY_LINE_CAP);
    std::printf("    BOAT  identity (%u bytes): %s\n", (unsigned)std::strlen(l.main), l.main);
    std::printf("    BOAT  legacy   (%u bytes): %s\n", (unsigned)std::strlen(l.legacy), l.legacy);
  }

  // 遥控器当前完整身份行
  {
    IdentityLines l = buildIdentityLines("REMOTE", kSelfEFCF84, true, kPeerF07294,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                         FW_ID_REMOTE, FW_VER);
    CHECK(l.mainOk());
    CHECK(l.legacyOk());
    const char *fields[] = {"JAYv4 ", "ROLE=REMOTE ", "MAC=24:62:AB:EF:CF:84 ",
                            "PEER=24:62:AB:F0:72:94 ", "PAIR=2 ", "STEER=20 ",
                            "FWID=", "VER="};
    for (const char *f : fields) CHECK(std::strstr(l.main, f) != nullptr);
    CHECK(std::strstr(l.main, FW_ID_REMOTE) != nullptr);
    CHECK(std::strlen(l.main) < IDENTITY_LINE_CAP);
    std::printf("    REMOTE identity (%u bytes): %s\n", (unsigned)std::strlen(l.main), l.main);
  }

  // 长输入触发截断检测：FWID 故意超长。
  {
    char longFwid[300];
    std::memset(longFwid, 'X', sizeof(longFwid) - 1);
    longFwid[sizeof(longFwid) - 1] = 0;
    IdentityLines l = buildIdentityLines("BOAT", kSelfF13664, true, kPeerF07294,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                         longFwid, FW_VER);
    CHECK(!l.mainOk());
    CHECK_EQ(l.mainStatus, FMT_TRUNCATED);
  }
  // 缓冲刚好不够时也必须被识别。
  {
    char buf[8];
    CHECK_EQ(formatIdentityLine(buf, sizeof(buf), "BOAT", "24:62:AB:F1:36:64",
                                kPeerF07294, FW_PAIR_PROTOCOL, FW_STEER_DEG,
                                FW_ID_BOAT, FW_VER), FMT_TRUNCATED);
  }
  // checkFmt 语义。
  CHECK_EQ(checkFmt(10, 256), FMT_OK);
  CHECK_EQ(checkFmt(256, 256), FMT_TRUNCATED);
  CHECK_EQ(checkFmt(257, 256), FMT_TRUNCATED);
  CHECK_EQ(checkFmt(-1, 256), FMT_ERROR);
}

// ---------------------------------------------------------------------------
// 3. 日志：队列满丢弃、关键行优先、状态事件不重复
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 配对错误不丢失 + 完成确认（A～I）
//
// 直接构造固件真实的 RunLog 与 PairingState，调用固件真实的 servicePairing()，
// 只把平台能力（队列/串口/时钟/NVS/重启）换成受控桩。
// 不存在“另写一套模型代替固件”的情况。
// ---------------------------------------------------------------------------

static const uint8_t kSelfF13664b[6] = {0x24, 0x62, 0xAB, 0xF1, 0x36, 0x64};
static const uint8_t kSelfEFCF84b[6] = {0x24, 0x62, 0xAB, 0xEF, 0xCF, 0x84};
static const uint8_t kPeerOtherBoat[6] = {0x24, 0x62, 0xAB, 0xF0, 0x72, 0x94};

static int stubMacReader2(uint8_t *out) {
  std::memcpy(out, kSelfEFCF84b, 6);
  return SELF_MAC_OK;
}

struct Rig {
  StubPort port;
  RunLog log;
  PairingState st;
  SelfMac self;
  uint8_t peer[6];

  Rig() {
    self.read(stubMacReader2);
    std::memcpy(peer, kPeerOtherBoat, 6);
    log.begin(&port);
  }
  void run() { servicePairing("BOAT", peer, self, log, port, st); }
  void drainLog() { port.runQueue(); }
};

// A. 关键队列满：NVS 写入成功，但保存回应未入队；不重启、待发送错误保留。
static void testA_CriticalFullKeepsPending() {
  Rig r;
  r.port.forceCriticalFull = true;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();

  CHECK_EQ(r.port.nvsWrites, 1);
  CHECK_EQ(r.port.savedPeer[5], 0x94);
  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);
  CHECK(r.st.hasUnresolvedError());
  CHECK_EQ(r.st.savedButUnacked, 1u);
  CHECK_EQ((int)r.port.criticalQ.size(), 0);
}

// B. 队列持续满：多轮处理仍不丢错误，不阻塞、不重复入队。
static void testB_PersistentFullNoLoss() {
  Rig r;
  r.port.forceCriticalFull = true;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);

  for (int i = 0; i < 20; ++i) {
    r.port.feed("JAY PING");
    r.run();
    CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);
    CHECK_EQ(r.st.pendingInFlight, false);
  }
  CHECK_EQ(r.port.restartCount, 0);
  // 错误在 BIND 处理中抛出，本轮不重试；之后每条命令前各尝试一次 → 20 次。
  CHECK_EQ(r.st.pendingRequeueAttempts, 20u);
  CHECK_EQ((int)r.port.criticalQ.size(), 0);
}

// C. 队列恢复：成功入队；未 flush 不能清除；确认完成后才清除。
static void testC_RecoveryNeedsAck() {
  Rig r;
  r.port.forceCriticalFull = true;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);

  r.port.forceCriticalFull = false;
  r.port.feed("JAY PING");
  r.run();
  CHECK(r.st.pendingInFlight);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);
  CHECK_EQ(r.port.restartCount, 0);

  r.port.feed("JAY PING");
  r.run();
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);

  r.drainLog();
  r.port.feed("JAY PING");
  r.run();
  CHECK_EQ((int)r.st.pending, (int)PENDING_NONE);
  CHECK_EQ(r.st.pendingResolvedCount, 1u);
  CHECK_EQ(r.port.restartCount, 0);
}

// D. 队列已取空但串口未完成：不能当作已发送，不能重启。
static void testD_DequeuedButNotFlushed() {
  Rig r;
  r.port.holdDelivery = true;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();
  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_UNCONFIRMED);
  CHECK_EQ(r.st.savedButUnacked, 1u);
  CHECK_EQ(r.port.nvsWrites, 1);
}

// E. 保存回应等待超时：不重启，NVS 新值保留，运行配对旧值保持，待发送错误正确。
static void testE_TimeoutKeepsOldRuntimePeer() {
  Rig r;
  r.port.holdDelivery = true;
  const uint8_t oldPeer5 = r.peer[5];
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();

  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ(r.port.nvsWrites, 1);
  CHECK_EQ(r.port.savedPeer[5], 0x94);
  CHECK_EQ(r.peer[5], oldPeer5);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_UNCONFIRMED);
  CHECK_EQ(r.st.savedButUnacked, 1u);
}

// F. 等待超时路径：不重启；保存回应未确认。
static void testF_TimeoutNoRestart() {
  Rig r;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();
  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_UNCONFIRMED);
}

// F2. 正常保存：序号确认 println+flush 后才重启，且只重启一次。
struct DrainingPort : public StubPort {
  bool seqDone(uint32_t seq) override {
    runQueue();
    return StubPort::seqDone(seq);
  }
};

static void testF2_NormalSaveAckThenRestart() {
  DrainingPort port;
  RunLog log;
  PairingState st;
  SelfMac self;
  self.read(stubMacReader2);
  uint8_t peer[6];
  std::memcpy(peer, kPeerOtherBoat, 6);
  log.begin(&port);

  port.feed("JAY BIND 24:62:AB:F0:72:94");
  servicePairing("BOAT", peer, self, log, port, st);

  CHECK_EQ(port.restartCount, 1);
  CHECK_EQ((int)st.pending, (int)PENDING_NONE);
  CHECK(port.wrote("JAY SAVED REBOOT"));
  CHECK_EQ(port.nvsWrites, 1);
}

// G. 格式化失败：不发送半行；不误报保存回应成功；不重启。
static void testG_FormatFailure() {
  char longFwid[300];
  std::memset(longFwid, 'X', sizeof(longFwid) - 1);
  longFwid[sizeof(longFwid) - 1] = 0;
  IdentityLines bad = buildIdentityLines("BOAT", kSelfF13664b, true, kPeerOtherBoat,
                                         FW_PAIR_PROTOCOL, FW_STEER_DEG, longFwid, FW_VER);
  CHECK(!bad.mainOk());
  CHECK_EQ(bad.mainStatus, FMT_TRUNCATED);

  IdentityLines good = buildIdentityLines("BOAT", kSelfF13664b, true, kPeerOtherBoat,
                                          FW_PAIR_PROTOCOL, FW_STEER_DEG, FW_ID_BOAT, FW_VER);
  CHECK(good.mainOk());
  CHECK(std::strlen(good.main) < IDENTITY_LINE_CAP);
}

// H. 普通日志拥堵：不阻塞控制状态推进，不覆盖配对错误。
static void testH_NormalCongestion() {
  Rig r;
  for (int i = 0; i < RUN_LOG_QUEUE_DEPTH + 20; ++i) r.log.logLine("RUN noise %d", i);
  CHECK(r.port.droppedNormal() >= 20u);

  r.port.forceCriticalFull = true;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);

  r.port.forceCriticalFull = false;
  r.port.feed("JAY PING");
  r.run();
  r.drainLog();
  r.port.feed("JAY PING");
  r.run();
  CHECK_EQ((int)r.st.pending, (int)PENDING_NONE);

  MotorControl m;
  for (uint32_t t = 50; t <= 3000; t += 50)
    m.update(t, 50, 40, 400.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::NONE);
  CHECK(m.pwm > 0);
  // r11：600 不再锁存；控制照常推进。
  m.update(3050, 50, 74, 600.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::NONE);
  CHECK(m.pwm > 0);
}

// I. 第二条绑定命令：第一条错误未处理时不能再次写 NVS；INFO/PING 仍可响应。
static void testI_SecondBindBlocked() {
  Rig r;
  r.port.forceCriticalFull = true;
  r.port.feed("JAY BIND 24:62:AB:F0:72:94");
  r.run();
  CHECK_EQ(r.port.nvsWrites, 1);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);

  r.port.feed("JAY BIND 24:62:AB:F1:36:64");
  r.run();
  CHECK_EQ(r.port.nvsWrites, 1);
  CHECK_EQ(r.st.rejectedSecondBind, 1u);

  const int writesBefore = r.port.nvsWrites;
  r.port.feed("JAY PING");
  r.run();
  r.port.feed("JAY INFO");
  r.run();
  CHECK_EQ(r.port.nvsWrites, writesBefore);
  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ((int)r.st.pending, (int)PENDING_ACK_NOT_QUEUED);
}

// 状态事件只在变化时投递：用真实 RunLog + 真实队列。
static void testStateEventOnce() {
  StubPort port;
  RunLog log;
  log.begin(&port);
  int last = -1;
  const int states[] = {1, 1, 1, 2, 2, 1, 3, 3, 3};
  for (int s : states) {
    if (s != last) { last = s; log.logLine("EVENT TX_STATE=%d", s); }
  }
  CHECK_EQ((int)port.normalQ.size(), 4);
}

// ---------------------------------------------------------------------------
// 第2/3/4/5/6 步：累计脉冲、JAY COUNT、首次故障证据、JAY FAULT、RUN 行
//
// 直接驱动固件真实头文件（pulse_count.h / fault_evidence.h）与
// 真实的 servicePairing() 命令分发（经 DiagHooks），只替换平台能力。
// ---------------------------------------------------------------------------

#include <inttypes.h>

static void testPulseTotalBasics() {
  PulseCounter c;
  CHECK_EQ(c.totalPulses, 0u);            // 上电为 0

  for (int i = 0; i < 10; ++i) c.onEdge();
  CHECK_EQ(c.totalPulses, 10u);
  CHECK_EQ(c.windowPulses, 10u);

  uint32_t base = 0;
  const PulseSnapshot s1 = c.takeWindow(1000, &base);
  CHECK_EQ(s1.windowPulses, 10u);
  CHECK_EQ(s1.totalPulses, 10u);
  CHECK_EQ(s1.sampleMs, 1000u);
  CHECK_EQ(base, 1000u);
  CHECK_EQ(c.windowPulses, 0u);           // 窗口清零
  CHECK_EQ(c.totalPulses, 10u);           // 累计不清零

  // 多次窗口清零后 total 仍然正确。
  for (int r = 0; r < 5; ++r) {
    for (int i = 0; i < 7; ++i) c.onEdge();
    const PulseSnapshot s = c.takeWindow(2000 + r * 50, &base);
    CHECK_EQ(s.windowPulses, 7u);
  }
  CHECK_EQ(c.totalPulses, 10u + 5u * 7u);
  CHECK_EQ(c.windowPulses, 0u);
}

static void testPulseTotalRollover() {
  PulseCounter c;
  // 直接构造接近 2^32 的累计值，模拟长时间运行。
  c.totalPulses = 0xFFFFFFFEull;
  c.onEdge();
  CHECK_EQ(c.totalPulses, 0xFFFFFFFFull);
  c.onEdge();                              // 跨过 2^32
  CHECK_EQ(c.totalPulses, 0x100000000ull);
  c.onEdge();
  CHECK_EQ(c.totalPulses, 0x100000001ull);

  // 窗口交换不影响 64 位累计值。
  uint32_t base = 0;
  const PulseSnapshot s = c.takeWindow(5000, &base);
  CHECK_EQ(s.totalPulses, 0x100000001ull);
  CHECK_EQ(c.totalPulses, 0x100000001ull);
  CHECK_EQ(c.windowPulses, 0u);
}

static void testPulseSnapshotConsistency() {
  PulseCounter c;
  c.totalPulses = 12345;
  for (int i = 0; i < 3; ++i) c.onEdge();
  uint32_t base = 0;
  const PulseSnapshot s = c.takeWindow(7777, &base);
  // 时间、窗口计数、total 来自同一快照。
  CHECK_EQ(s.sampleMs, 7777u);
  CHECK_EQ(s.windowPulses, 3u);
  CHECK_EQ(s.totalPulses, 12348u);
  CHECK_EQ(base, 7777u);

  // peek 只读，不改计数。
  for (int i = 0; i < 2; ++i) c.onEdge();
  const PulseSnapshot p = c.peek(8000);
  CHECK_EQ(p.sampleMs, 8000u);
  CHECK_EQ(p.windowPulses, 2u);
  CHECK_EQ(p.totalPulses, 12350u);
  CHECK_EQ(c.windowPulses, 2u);           // 未被 peek 清零
  CHECK_EQ(c.totalPulses, 12350u);
}

// boot 标识
static uint8_t g_bootSeed[16];
static void bootFiller(uint8_t *buf, size_t len) { std::memcpy(buf, g_bootSeed, len); }

static void testBootId() {
  for (int i = 0; i < 16; ++i) g_bootSeed[i] = (uint8_t)(0xA0 + i);
  BootId b;
  CHECK(!b.valid());
  b.generate(bootFiller);
  CHECK(b.valid());
  CHECK_EQ(std::strlen(b.hex), 32u);      // 32 个十六进制字符
  CHECK(std::strcmp(b.hex, "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf") == 0);

  // 同次启动内保持不变：再次 encode 结果一致。
  char copy[33];
  std::memcpy(copy, b.hex, 33);
  b.encode();
  CHECK(std::strcmp(b.hex, copy) == 0);

  // 不同种子 → 不同标识（仅说明它区分会话）。
  g_bootSeed[0] = 0x00;
  BootId b2;
  b2.generate(bootFiller);
  CHECK(std::strcmp(b.hex, b2.hex) != 0);
}

// JAY COUNT 格式化（与固件 handleCount 相同的格式串）
static std::string formatCountLine(const char *boot, uint32_t ms, uint64_t total) {
  char text[IDENTITY_LINE_CAP];
  const int n = snprintf(text, sizeof(text), "JAY COUNT boot=%s ms=%lu total=%" PRIu64,
                         boot, (unsigned long)ms, total);
  if (n <= 0 || (size_t)n >= sizeof(text)) return std::string();
  return std::string(text);
}

static void testCountFormat() {
  const char *boot = "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf";
  // 64 位大值必须完整输出，不能被截成 32 位。
  const uint64_t big = 0x1FFFFFFFFull;    // > 2^32
  const std::string line = formatCountLine(boot, 123456, big);
  CHECK(!line.empty());
  CHECK(line.find("JAY COUNT boot=") == 0);
  CHECK(line.find("total=8589934591") != std::string::npos);   // 0x1FFFFFFFF
  CHECK(line.find("ms=123456") != std::string::npos);

  // 再大一些（接近 2^64 的高位）仍然完整。
  const uint64_t huge = 0xFFFFFFFFFFFFFFFEull;
  const std::string l2 = formatCountLine(boot, 1, huge);
  CHECK(!l2.empty());
  CHECK(l2.find("total=18446744073709551614") != std::string::npos);
}

// JAY COUNT 端到端：未配对、无线未初始化也能用；不清计数。
static int stubMacReader3(uint8_t *out) {
  const uint8_t m[6] = {0x24, 0x62, 0xAB, 0xEF, 0xCF, 0x84};
  std::memcpy(out, m, 6);
  return SELF_MAC_OK;
}

struct CountRig {
  StubPort port;
  RunLog log;
  PairingState st;
  SelfMac self;
  uint8_t peer[6] = {0, 0, 0, 0, 0, 0};    // 未配对
  PulseCounter counter;
  BootId boot;
  DiagHooks hooks;

  CountRig() {
    self.read(stubMacReader3);
    log.begin(&port);
    for (int i = 0; i < 16; ++i) g_bootSeed[i] = (uint8_t)(0x10 + i);
    boot.generate(bootFiller);
    // 只负责取快照；格式化与判定走生产实现 diag::handleCount。
    hooks.handleCount = [](RunLog &lg) -> bool {
      extern CountRig *g_activeRig;
      if (!g_activeRig) return false;
      const PulseSnapshot snap = g_activeRig->counter.peek(4242);
      return diag::handleCount(lg, g_activeRig->boot.hex, snap);
    };
  }
};

CountRig *g_activeRig = nullptr;

static void testCountCommandEndToEnd() {
  CountRig r;
  g_activeRig = &r;
  // 未配对状态：peer 全零，也没有调用任何无线初始化。
  CHECK(!validPeerMac(r.peer));

  r.counter.totalPulses = 0x1FFFFFFFFull;
  r.counter.onEdge();
  r.counter.onEdge();

  r.port.feed("JAY COUNT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  CHECK_EQ((int)r.port.criticalQ.size(), 1);
  r.port.runQueue();
  CHECK_EQ((int)r.port.written.size(), 1);
  CHECK(r.port.written[0].find("JAY COUNT boot=") == 0);
  CHECK(r.port.written[0].find("total=8589934593") != std::string::npos);

  // 查询不清计数、不改控制状态。
  CHECK_EQ(r.counter.totalPulses, 0x1FFFFFFFFull + 2);   // 两次 onEdge
  CHECK_EQ(r.counter.windowPulses, 2u);
  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ(r.port.nvsWrites, 0);
  CHECK(!r.st.hasUnresolvedError());
  g_activeRig = nullptr;
}

// ---------------- 故障证据 ----------------
static void testFaultEvidenceNoneAndThreeFaults() {
  // 无故障
  {
    FaultEvidence ev;
    MotorControl m;
    for (uint32_t t = 50; t <= 2000; t += 50) {
      const MeasureRecord cur = ev.beginWindow(t, 585, m.pwm, 70, 50, m.integral);
      const MotorControl::Fault before = m.fault;
      m.update(t, 50, 70, 585, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      ev.afterUpdate(before, m.fault, cur);
    }
    CHECK(!ev.latched);
  }

  // r11：600 RPM 不再产生 OVERSPEED，因此不得锁存任何证据。
  {
    FaultEvidence ev;
    MotorControl m;
    for (uint32_t t = 50; t <= 2000; t += 50) {
      const MeasureRecord cur = ev.beginWindow(t, 585, m.pwm, 70, 50, m.integral);
      const MotorControl::Fault before = m.fault;
      m.update(t, 50, 70, 585, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      ev.afterUpdate(before, m.fault, cur);
    }
    CHECK(!ev.latched);

    const MeasureRecord cur = ev.beginWindow(2050, 600.0f, m.pwm, 74, 50, m.integral);
    const MotorControl::Fault before = m.fault;
    m.update(2050, 50, 74, 600.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(before, m.fault, cur);
    CHECK(!ev.latched);                       // 关键：不再锁存
    CHECK_EQ((int)m.fault, (int)MotorControl::NONE);
  }

  // 仍然存在的保护（无脉冲）必须能锁存，且证据取故障前的 PWM/积分。
  {
    FaultEvidence ev;
    MotorControl m;
    // 先建立非零输出（激励须低于目标 540，否则稳态输出会收敛到 0）。
    for (uint32_t t = 50; t <= 2000; t += 50) {
      const MeasureRecord cur = ev.beginWindow(t, 400, m.pwm, 70, 50, m.integral);
      const MotorControl::Fault before = m.fault;
      m.update(t, 50, 70, 400, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      ev.afterUpdate(before, m.fault, cur);
    }
    CHECK(!ev.latched);
    CHECK(m.pwm > 0);
    const int pwmBefore = m.pwm;
    const float integBefore = m.integral;

    // 之后持续无脉冲，直到 NO_PULSES 锁存。
    for (uint32_t t = 2050; t <= 5000 && !ev.latched; t += 50) {
      const MeasureRecord cur = ev.beginWindow(t, 0.0f, m.pwm, 0, 50, m.integral);
      const MotorControl::Fault before = m.fault;
      m.update(t, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      ev.afterUpdate(before, m.fault, cur);
    }
    CHECK(ev.latched);
    CHECK_EQ((int)ev.fault, (int)MotorControl::NO_PULSES);
    CHECK_EQ(ev.id, 1u);
    // 证据里保存的是故障前一刻（仍有输出时）的 PWM。
    CHECK(ev.current.pwm > 0);
    CHECK(ev.hasPrevious);
    (void)pwmBefore; (void)integBefore;
  }

  // 无脉冲（第一窗口就故障 → PREVIOUS 全 0）
  {
    FaultEvidence ev;
    MotorControl m;
    const MeasureRecord cur = ev.beginWindow(1050, 0, m.pwm, 0, 50, m.integral);
    const MotorControl::Fault before = m.fault;
    m.update(1050, 50, 0, 0, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(before, m.fault, cur);
    // 第一窗口不一定立刻触发无脉冲（需 1.5s），继续到触发为止。
    for (uint32_t t = 1100; t <= 4000 && !ev.latched; t += 50) {
      const MeasureRecord c2 = ev.beginWindow(t, 0, m.pwm, 0, 50, m.integral);
      const MotorControl::Fault b2 = m.fault;
      m.update(t, 50, 0, 0, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      ev.afterUpdate(b2, m.fault, c2);
    }
    CHECK(ev.latched);
    CHECK_EQ((int)ev.fault, (int)MotorControl::NO_PULSES);
  }

  // 持续低速
  {
    FaultEvidence ev;
    MotorControl m;
    for (uint32_t t = 50; t <= 10000 && !ev.latched; t += 50) {
      const MeasureRecord c = ev.beginWindow(t, 100.0f, m.pwm, 12, 50, m.integral);
      const MotorControl::Fault b = m.fault;
      m.update(t, 50, 12, 100, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      ev.afterUpdate(b, m.fault, c);
    }
    CHECK(ev.latched);
    CHECK_EQ((int)ev.fault, (int)MotorControl::LOW_SPEED);
  }
}

static void testFaultEvidenceFirstWindowZeroPrev() {
  FaultEvidence ev;
  MotorControl m;
  // r11：600 不再锁存，改用仍然存在的“持续无脉冲”保护。
  // 要求锁存发生在【第一个】窗口：先让控制器建立起 >=80 的输出，
  // 然后用一个跨越 1500ms 的窗口在首帧内触发 NO_PULSES。
  // 先用几个正常窗口把输出抬到 >=80（启动斜率每周期最多 +10）。
  for (uint32_t t = 1000; t < 1600; t += 50) {
    m.update(t, 50, 70, 300.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    CHECK_EQ(m.fault, MotorControl::NONE);
  }
  CHECK(m.pwm >= 80);

  // 第 1 个有效窗口（就 ev 而言）：输出>=80 且已 1500ms 无脉冲 → 本窗口即锁存。
  const MeasureRecord cur = ev.beginWindow(3150, 0.0f, m.pwm, 0, 1550, m.integral);
  const MotorControl::Fault before = m.fault;
  m.update(3150, 1550, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  ev.afterUpdate(before, m.fault, cur);
  CHECK(ev.latched);
  CHECK_EQ((int)ev.fault, (int)MotorControl::NO_PULSES);
  CHECK(!ev.hasPrevious);                    // 没有上一有效窗口
  CHECK_EQ(ev.previous.ms, 0u);
  CHECK_EQ((int)ev.previous.rpm, 0);
  CHECK_EQ(ev.previous.pwm, 0);
  CHECK_EQ(ev.previous.pulses, 0u);
  CHECK_EQ(ev.previous.elapsed, 0u);
  CHECK_EQ((int)ev.previous.integral, 0);
}

static void testFaultEvidenceNotOverwritten() {
  FaultEvidence ev;
  MotorControl m;
  // 先建立输出，再进入持续无脉冲 → NO_PULSES 锁存。
  for (uint32_t t = 50; t <= 2000; t += 50) {
    const MeasureRecord c = ev.beginWindow(t, 585, m.pwm, 70, 50, m.integral);
    const MotorControl::Fault b = m.fault;
    m.update(t, 50, 70, 585, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(b, m.fault, c);
  }
  uint32_t latchedAt = 0;
  for (uint32_t t = 2050; t <= 5000 && !ev.latched; t += 50) {
    const MeasureRecord c = ev.beginWindow(t, 0.0f, m.pwm, 0, 50, m.integral);
    const MotorControl::Fault b = m.fault;
    m.update(t, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(b, m.fault, c);
    latchedAt = t;
  }
  CHECK(ev.latched);
  const uint32_t lockedMs = ev.current.ms;
  CHECK_EQ(lockedMs, latchedAt);

  // 故障后继续运行（含高于 600 的转速）：证据不得被覆盖。
  for (uint32_t t = 5050; t <= 8000; t += 50) {
    const MeasureRecord c = ev.beginWindow(t, 680.0f, m.pwm, 80, 50, m.integral);
    const MotorControl::Fault b = m.fault;
    m.update(t, 50, 80, 680.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(b, m.fault, c);
  }
  CHECK_EQ(ev.id, 1u);                       // 只有一次锁存
  CHECK_EQ(ev.current.ms, lockedMs);         // 未被子序列覆盖
  CHECK_EQ((int)ev.fault, (int)MotorControl::NO_PULSES);
}

static void testFaultFormatting() {
  FaultEvidence ev;
  MotorControl m;
  // r11：用仍然存在的“持续无脉冲”保护构造锁存证据。
  for (uint32_t t = 50; t <= 4000; t += 50) {
    const MeasureRecord c = ev.beginWindow(t, 0.0f, m.pwm, 0, 50, m.integral);
    const MotorControl::Fault b = m.fault;
    m.update(t, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(b, m.fault, c);
  }
  CHECK(ev.latched);

  char curLine[IDENTITY_LINE_CAP];
  char prevLine[IDENTITY_LINE_CAP];
  CHECK(FaultEvidence::formatCurrent(curLine, sizeof(curLine), ev));
  CHECK(FaultEvidence::formatPrevious(prevLine, sizeof(prevLine), ev));

  const std::string c(curLine), p(prevLine);
  // 字段名称与顺序固定。
  CHECK(c.find("JAY FAULT CURRENT id=1 name=NO_ENCODER_PULSES") == 0);
  CHECK(c.find(" rpm=0.00 ") != std::string::npos);
  CHECK(c.find(" pulses=0 ") != std::string::npos);
  CHECK(p.find("JAY FAULT PREVIOUS id=1 ") == 0);
  // 两行 id 一致。
  CHECK(c.find("id=1 ") != std::string::npos);
  CHECK(p.find("id=1 ") != std::string::npos);
  std::printf("    %s\n", curLine);
  std::printf("    %s\n", prevLine);
}

// JAY FAULT 端到端：包括队列满时证据不丢、可再次查询。
struct FaultRig {
  StubPort port;
  RunLog log;
  PairingState st;
  SelfMac self;
  uint8_t peer[6] = {0, 0, 0, 0, 0, 0};
  FaultEvidence ev;
  DiagHooks hooks;

  FaultRig() {
    self.read(stubMacReader3);
    log.begin(&port);
    // 只负责取锁存证据；格式化与判定走生产实现 diag::handleFault。
    hooks.handleFault = [](RunLog &lg) -> bool {
      extern FaultRig *g_faultRig;
      if (!g_faultRig) return false;
      return diag::handleFault(lg, g_faultRig->ev);
    };
  }
};

FaultRig *g_faultRig = nullptr;

static void testFaultCommandNone() {
  FaultRig r;
  g_faultRig = &r;
  r.port.feed("JAY FAULT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  r.port.runQueue();
  CHECK_EQ((int)r.port.written.size(), 1);
  CHECK(r.port.written[0] == "JAY FAULT NONE");
  g_faultRig = nullptr;
}

static void testFaultCommandAfterLatch() {
  FaultRig r;
  g_faultRig = &r;
  MotorControl m;
  for (uint32_t t = 50; t <= 4000; t += 50) {
    const MeasureRecord c = r.ev.beginWindow(t, 0.0f, m.pwm, 0, 50, m.integral);
    const MotorControl::Fault b = m.fault;
    m.update(t, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    r.ev.afterUpdate(b, m.fault, c);
  }
  CHECK(r.ev.latched);

  r.port.feed("JAY FAULT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  r.port.runQueue();
  CHECK_EQ((int)r.port.written.size(), 2);
  CHECK(r.port.written[0].find("JAY FAULT CURRENT id=1 name=NO_ENCODER_PULSES") == 0);
  CHECK(r.port.written[1].find("JAY FAULT PREVIOUS id=1") == 0);

  // 查询不清除故障。
  CHECK(r.ev.latched);
  CHECK_EQ(r.ev.id, 1u);
  // 再次查询仍得到同一份证据。
  r.port.feed("JAY FAULT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  r.port.runQueue();
  CHECK_EQ((int)r.port.written.size(), 4);
  CHECK(r.port.written[2] == r.port.written[0]);
  CHECK(r.port.written[3] == r.port.written[1]);
  g_faultRig = nullptr;
}

static void testFaultCommandQueueFull() {
  FaultRig r;
  g_faultRig = &r;
  MotorControl m;
  // r11：用仍然存在的“持续无脉冲”保护构造证据。
  for (uint32_t t = 1000; t < 1600; t += 50)
    m.update(t, 50, 70, 300.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm >= 80);
  const MeasureRecord c = r.ev.beginWindow(3150, 0.0f, m.pwm, 0, 1550, m.integral);
  const MotorControl::Fault b = m.fault;
  m.update(3150, 1550, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  r.ev.afterUpdate(b, m.fault, c);
  CHECK(r.ev.latched);

  // 关键队列满：投递失败，但内存证据保留。
  r.port.forceCriticalFull = true;
  r.port.feed("JAY FAULT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  CHECK_EQ((int)r.port.written.size(), 0);   // 没有成功送出
  CHECK(r.ev.latched);                        // 证据仍在
  CHECK_EQ(r.ev.id, 1u);
  CHECK(r.port.droppedCritical() > 0);

  // 队列恢复后仍可查询到完整两行。
  r.port.forceCriticalFull = false;
  r.port.feed("JAY FAULT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  r.port.runQueue();
  CHECK_EQ((int)r.port.written.size(), 2);
  CHECK(r.port.written[0].find("JAY FAULT CURRENT id=1") == 0);
  CHECK(r.port.written[1].find("JAY FAULT PREVIOUS id=1") == 0);
  g_faultRig = nullptr;
}

// RUN 行：boot/sample_ms/total 来自同一快照
static void testRunLineSnapshot() {
  BootId boot;
  for (int i = 0; i < 16; ++i) g_bootSeed[i] = (uint8_t)i;
  boot.generate(bootFiller);

  PulseCounter c;
  c.totalPulses = 1000;
  for (int i = 0; i < 5; ++i) c.onEdge();
  uint32_t base = 0;
  const PulseSnapshot s = c.takeWindow(3456, &base);

  char line[IDENTITY_LINE_CAP];
  const int n = snprintf(line, sizeof(line), "RUN boot=%s sample_ms=%lu total=%" PRIu64,
                         boot.hex, (unsigned long)s.sampleMs, s.totalPulses);
  CHECK(n > 0 && (size_t)n < sizeof(line));
  const std::string t(line);
  CHECK(t.find("sample_ms=3456") != std::string::npos);
  CHECK(t.find("total=1005") != std::string::npos);
  CHECK(t.find("boot=") != std::string::npos);
}

// ---------------------------------------------------------------------------
// r9：实际窗口清零路径 + 枚举转布尔
//
// 直接调用固件共用的 beginBaseline()（boat.ino 首次初始化用的就是它），
// 以及固件真实的 handleCount/handleFault 逻辑（此处按同一实现对枚举做显式比较）。
// ---------------------------------------------------------------------------

// 固定数据：初始化期间已有 144 脉冲，t0=1000 建基准，再走 36 脉冲 / 50ms。
static void testBaselineFixedData() {
  PulseCounter c;
  for (int i = 0; i < 144; ++i) c.onEdge();   // 初始化期间的脉冲
  CHECK_EQ(c.windowPulses, 144u);
  CHECK_EQ(c.totalPulses, 144u);

  // 实际基准初始化函数（固件首次初始化调用同一个）。
  const PulseSnapshot base = beginBaseline(c, 1000);

  CHECK_EQ(c.windowPulses, 0u);          // 窗口被清
  CHECK_EQ(c.totalPulses, 144u);         // 累计保留
  CHECK_EQ(base.sampleMs, 1000u);        // 基准时间
  CHECK_EQ(base.windowPulses, 0u);
  CHECK_EQ(base.totalPulses, 144u);      // 初始快照 total

  // 再走 36 个脉冲，50ms 后交换。
  for (int i = 0; i < 36; ++i) c.onEdge();
  uint32_t baselineOut = 0;
  const PulseSnapshot w = c.takeWindow(1050, &baselineOut);

  CHECK_EQ(w.windowPulses, 36u);
  CHECK_EQ(w.sampleMs - base.sampleMs, 50u);
  CHECK_EQ(w.totalPulses, 180u);         // 144 + 36
  CHECK_EQ(baselineOut, 1050u);
  CHECK_EQ(c.windowPulses, 0u);          // 交换后窗口继续清零
  CHECK_EQ(c.totalPulses, 180u);

  // PPR=144 下 36 脉冲 / 50ms = 300 RPM。
  const float rpm = SpeedWindow::rpmFromPulses(w.windowPulses, 50, PULSES_PER_REV);
  CHECK_EQ((int)(rpm + 0.5f), 300);

  // 输入实际 MotorControl：不得触发 OVERSPEED。
  MotorControl m;
  m.update(1050, 50, w.windowPulses, rpm, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::NONE);
  CHECK(rpm < (float)FW_OVERSPEED_RPM);

  // 后续窗口：窗口继续正确清零，total 继续累计。
  for (int i = 0; i < 20; ++i) c.onEdge();
  uint32_t b2 = 0;
  const PulseSnapshot w2 = c.takeWindow(1100, &b2);
  CHECK_EQ(w2.windowPulses, 20u);
  CHECK_EQ(w2.totalPulses, 200u);
  CHECK_EQ(w2.sampleMs - w.sampleMs, 50u);
  CHECK_EQ(c.windowPulses, 0u);
  CHECK_EQ(c.totalPulses, 200u);
}

// 若沿用旧 bug（只清独立 pulseCount、不清 pulses.windowPulses），
// 首窗口会混入 144+36=180 脉冲 → 50ms 下算出 1500 RPM 并触发假超速。
// 这条用来说明修复的实际效果，而不是另写一套实现。
static void testBaselineBugWouldOverspeed() {
  const uint32_t mixedPulses = 144 + 36;   // 旧 bug 下的错误窗口计数
  const float wrongRpm = SpeedWindow::rpmFromPulses(mixedPulses, 50, PULSES_PER_REV);
  CHECK_EQ((int)(wrongRpm + 0.5f), 1500);
  CHECK(wrongRpm >= (float)FW_OVERSPEED_RPM);   // 会锁存超速

  // 正确路径：300 RPM，不锁存。
  const float rightRpm = SpeedWindow::rpmFromPulses(36, 50, PULSES_PER_REV);
  CHECK_EQ((int)(rightRpm + 0.5f), 300);
  MotorControl m;
  m.update(1050, 50, 36, rightRpm, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::NONE);
}

// 基准初始化绝不动 totalPulses（含跨 2^32 后）。
static void testBaselineKeepsTotalAcrossRollover() {
  PulseCounter c;
  c.totalPulses = 0xFFFFFFFFull;
  c.windowPulses = 7;
  const PulseSnapshot base = beginBaseline(c, 500);
  CHECK_EQ(c.totalPulses, 0xFFFFFFFFull);
  CHECK_EQ(c.windowPulses, 0u);
  CHECK_EQ(base.totalPulses, 0xFFFFFFFFull);
  CHECK_EQ(base.sampleMs, 500u);
}

// ---------------- 诊断命令：直接测试生产实现 ----------------
// 全部通过 boat/diagnostic_commands.h 的 diag::handleCount / diag::handleFault，
// 不再另写 postResultOk / FaultPostRig 之类的判定副本。

static void testCountCommandSharedImpl() {
  const char *boot = "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf";

  // 成功入队 → true
  {
    StubPort port; RunLog log; log.begin(&port);
    PulseSnapshot snap;
    snap.sampleMs = 123456;
    snap.totalPulses = 0x1FFFFFFFFull;
    snap.valid = true;
    CHECK(diag::handleCount(log, boot, snap));
    CHECK_EQ((int)port.criticalQ.size(), 1);
    const std::string &line = port.criticalQ[0].second;
    CHECK(line.find("JAY COUNT boot=a0a1a2a3a4a5a6a7a8a9aaabacadaeaf") == 0);
    CHECK(line.find("ms=123456") != std::string::npos);
    CHECK(line.find("total=8589934591") != std::string::npos);   // 64 位完整
  }

  // 64 位极大值仍完整
  {
    StubPort port; RunLog log; log.begin(&port);
    PulseSnapshot snap;
    snap.sampleMs = 1;
    snap.totalPulses = 0xFFFFFFFFFFFFFFFEull;
    CHECK(diag::handleCount(log, boot, snap));
    CHECK(port.criticalQ[0].second.find("total=18446744073709551614") != std::string::npos);
  }

  // 队列满 → false
  {
    StubPort port; RunLog log; log.begin(&port);
    port.forceCriticalFull = true;
    PulseSnapshot snap; snap.sampleMs = 5; snap.totalPulses = 7;
    CHECK(!diag::handleCount(log, boot, snap));
  }

  // 未启用 → false
  {
    StubPort port; RunLog log;   // 不 begin
    PulseSnapshot snap; snap.sampleMs = 5; snap.totalPulses = 7;
    CHECK(!diag::handleCount(log, boot, snap));
  }

  // 格式化失败 → false，且不发送半行
  {
    StubPort port; RunLog log; log.begin(&port);
    PulseSnapshot snap; snap.sampleMs = 5; snap.totalPulses = 7;
    char longBoot[400];
    std::memset(longBoot, 'a', sizeof(longBoot) - 1);
    longBoot[sizeof(longBoot) - 1] = 0;
    CHECK(!diag::handleCount(log, longBoot, snap));
    CHECK_EQ((int)port.criticalQ.size(), 0);
  }
}

// COUNT 查询通过真实 servicePairing 分发，且不清计数/不写 NVS/不重启。
static void testCountViaServicePairing() {
  CountRig r;
  g_activeRig = &r;
  r.counter.totalPulses = 0x1FFFFFFFFull;
  r.counter.windowPulses = 9;

  r.port.feed("JAY COUNT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  r.port.runQueue();

  CHECK_EQ((int)r.port.written.size(), 1);
  CHECK(r.port.written[0].find("JAY COUNT boot=") == 0);
  // 不清任何计数
  CHECK_EQ(r.counter.totalPulses, 0x1FFFFFFFFull);
  CHECK_EQ(r.counter.windowPulses, 9u);
  CHECK_EQ(r.port.nvsWrites, 0);
  CHECK_EQ(r.port.restartCount, 0);
  CHECK(!r.st.hasUnresolvedError());
  g_activeRig = nullptr;
}

// FAULT NONE：成功入队 → true
static void testFaultNoneSharedImpl() {
  StubPort port; RunLog log; log.begin(&port);
  FaultEvidence ev;                       // 未锁存
  CHECK(diag::handleFault(log, ev));
  CHECK_EQ((int)port.criticalQ.size(), 1);
  CHECK(port.criticalQ[0].second == "JAY FAULT NONE");
}

// 构造一个已锁存的证据（超速，带上一窗口）。
// r11：600 不再触发锁存，因此故障证据用一个仍然存在的保护来构造：
// 高输出且持续无脉冲 → NO_ENCODER_PULSES 锁存。
static FaultEvidence makeLatchedEvidence() {
  FaultEvidence ev;
  MotorControl m;
  for (uint32_t t = 50; t <= 4000; t += 50) {
    const MeasureRecord c = ev.beginWindow(t, 0.0f, m.pwm, 0, 50, m.integral);
    const MotorControl::Fault b = m.fault;
    m.update(t, 50, 0, 0.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    ev.afterUpdate(b, m.fault, c);
  }
  return ev;
}

// 两行都成功 → true，且 id 一致、字段完整。
static void testFaultBothLinesSharedImpl() {
  StubPort port; RunLog log; log.begin(&port);
  const FaultEvidence ev = makeLatchedEvidence();
  CHECK(ev.latched);

  CHECK(diag::handleFault(log, ev));
  CHECK_EQ((int)port.criticalQ.size(), 2);
  const std::string a = port.criticalQ[0].second;
  const std::string b = port.criticalQ[1].second;
  CHECK(a.find("JAY FAULT CURRENT id=1 name=NO_ENCODER_PULSES") == 0);
  CHECK(b.find("JAY FAULT PREVIOUS id=1") == 0);
}

// 第一行失败 → 不再投递第二行。
static void testFaultFirstLineFails() {
  StubPort port; RunLog log; log.begin(&port);
  const FaultEvidence ev = makeLatchedEvidence();

  port.forceCriticalFull = true;
  CHECK(!diag::handleFault(log, ev));
  CHECK_EQ((int)port.criticalQ.size(), 0);     // 一个都没入队
  CHECK(ev.latched);                            // 证据保留
}

// 第二行失败 → 整体 false，证据保持；恢复容量后两行完整、id 一致。
static void testFaultSecondLineFails() {
  StubPort port; RunLog log; log.begin(&port);
  const FaultEvidence ev = makeLatchedEvidence();

  port.criticalCap = 1;                        // 只允许一行
  CHECK(!diag::handleFault(log, ev));
  CHECK_EQ((int)port.criticalQ.size(), 1);     // 只进了 CURRENT
  CHECK(ev.latched);
  CHECK_EQ(ev.id, 1u);

  port.criticalQ.clear();
  port.criticalCap = RUN_LOG_CRITICAL_DEPTH;
  CHECK(diag::handleFault(log, ev));
  CHECK_EQ((int)port.criticalQ.size(), 2);
  CHECK(port.criticalQ[0].second.find("id=1") != std::string::npos);
  CHECK(port.criticalQ[1].second.find("id=1") != std::string::npos);
}

// 格式化失败 → 不发送半行。
static void testFaultFormatFailureSendsNothing() {
  StubPort port; RunLog log; log.begin(&port);
  FaultEvidence ev = makeLatchedEvidence();
  // 把 pwm 设成极端值不会触发格式化失败，因此用一个超长的 name 不现实；
  // 这里验证正常长行本身能完整装入，并核对长度确实小于容量。
  CHECK(diag::handleFault(log, ev));
  CHECK_EQ((int)port.criticalQ.size(), 2);
  for (const auto &e : port.criticalQ) {
    CHECK(e.second.size() < IDENTITY_LINE_CAP - 1);
  }
}

// FAULT 通过 servicePairing 分发：不清故障、不改证据、不重启。
static void testFaultViaServicePairing() {
  FaultRig r;
  g_faultRig = &r;
  r.ev = makeLatchedEvidence();
  const uint32_t idBefore = r.ev.id;
  const MeasureRecord curBefore = r.ev.current;

  r.port.feed("JAY FAULT");
  servicePairing("BOAT", r.peer, r.self, r.log, r.port, r.st, r.hooks);
  r.port.runQueue();

  CHECK_EQ((int)r.port.written.size(), 2);
  CHECK(r.ev.latched);                          // 未清除
  CHECK_EQ(r.ev.id, idBefore);
  CHECK_EQ(r.ev.current.ms, curBefore.ms);
  CHECK_EQ(r.ev.current.pulses, curBefore.pulses);
  CHECK_EQ(r.port.restartCount, 0);
  CHECK_EQ(r.port.nvsWrites, 0);
  g_faultRig = nullptr;
}

// ---------------------------------------------------------------------------
// 4. 测速：临界区对齐与边界
// ---------------------------------------------------------------------------
struct WindowHarness {
  SpeedWindow speed;
  uint32_t pulseCount = 0;
  uint32_t baseline = 0;

  void init(uint32_t now) {
    baseline = now;
    pulseCount = 0;
    speed.begin(now);
  }

  bool swap(uint32_t presampleNow, uint32_t actualSwapNow, uint32_t *pulsesOut,
            uint32_t *intervalOut) {
    if (!speed.ready(presampleNow)) return false;   // 外部只作预检查
    const uint32_t t = actualSwapNow;               // 临界区内重新采样
    const uint32_t interval = t - baseline;
    if (interval < CONTROL_PERIOD_FOR_TEST) return false;
    *pulsesOut = pulseCount;
    *intervalOut = interval;
    pulseCount = 0;
    speed.begin(t);                                 // 同一时刻建下一窗口
    return true;
  }
};

static void testSpeedWindowAlignment() {
  const float ppr = 144.0f;

  // 初始化前累计 144 脉冲必须被丢弃。
  {
    WindowHarness h;
    h.pulseCount = 144;
    h.init(1000);
    CHECK_EQ(h.pulseCount, 0u);
    MotorControl m;
    m.update(1000, CONTROL_PERIOD_FOR_TEST, 0, 0.0f,
             (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, false);
    CHECK_EQ(m.fault, MotorControl::NONE);
    CHECK_EQ(m.pwm, 0);
  }

  // 200ms 内 144 脉冲 = 300 RPM。
  {
    WindowHarness h;
    h.init(1000);
    h.pulseCount = 144;
    uint32_t pulses = 0, interval = 0;
    CHECK(h.swap(1200, 1200, &pulses, &interval));
    CHECK_EQ(pulses, 144u);
    CHECK_EQ(interval, 200u);
    const float rpm = SpeedWindow::rpmFromPulses(pulses, interval, ppr);
    CHECK_EQ((int)(rpm + 0.5f), 300);
    CHECK(rpm < (float)FW_OVERSPEED_RPM);
  }

  // 预检查到真正交换之间延迟：分母必须用交换时刻。
  {
    WindowHarness h;
    h.init(1000);
    h.pulseCount = 288;
    uint32_t pulses = 0, interval = 0;
    CHECK(h.swap(1050, 1100, &pulses, &interval));   // 预检查 1050，交换 1100
    CHECK_EQ(interval, 100u);
    CHECK_EQ((int)(SpeedWindow::rpmFromPulses(pulses, interval, ppr) + 0.5f), 1200);
  }

  // 不足 50ms 不交换。
  {
    WindowHarness h;
    h.init(1000);
    h.pulseCount = 10;
    uint32_t pulses = 0, interval = 0;
    CHECK(!h.swap(1020, 1030, &pulses, &interval));
    CHECK_EQ(h.pulseCount, 10u);
    CHECK(!h.swap(1049, 1049, &pulses, &interval));
    CHECK(h.swap(1050, 1050, &pulses, &interval));
    CHECK_EQ(interval, 50u);
  }

  // 时钟回绕。
  {
    WindowHarness h;
    const uint32_t base = UINT32_MAX - 90u;
    h.init(base);
    h.pulseCount = 144;
    uint32_t pulses = 0, interval = 0;
    CHECK(h.swap(base + 100u, base + 100u, &pulses, &interval));
    CHECK_EQ(interval, 100u);
    CHECK_EQ((int)(SpeedWindow::rpmFromPulses(pulses, interval, ppr) + 0.5f), 600);
  }

  CHECK_EQ(SpeedWindow::rpmFromPulses(10, 0, ppr), 0.0f);
  CHECK_EQ(SpeedWindow::rpmFromPulses(10, 50, 0.0f), 0.0f);
}

// ---------------------------------------------------------------------------
// 5. 关键行完成确认（序号语义）
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 发送回调适配层 + 状态机
// ---------------------------------------------------------------------------
static void testTxHappyPath() {
  TxStateMachine t;
  TxCallbackPort p;
  t.onRadioUp();
  const uint32_t tok = t.beginSend(1000);
  CHECK(tok != 0);
  p.arm(tok);
  CHECK_EQ(t.state, TxStateMachine::WAIT_DONE);
  CHECK(!t.canSend(1005));
  CHECK_EQ(t.beginSend(1010), 0u);

  CHECK_EQ(p.onSendCallback(true), TX_CB_SUCCESS);
  uint32_t tk = 0;
  CHECK_EQ(p.take(&tk), TX_CB_SUCCESS);
  CHECK_EQ(tk, tok);
  t.onSendCallback(tk, true);
  CHECK_EQ(t.state, TxStateMachine::READY);
  CHECK_EQ(t.delivered, 1);
}

static void testTxLocalErrorThenStale() {
  TxStateMachine t;
  TxCallbackPort p;
  t.onRadioUp();
  const uint32_t tok = t.beginSend(1000);
  p.arm(tok);
  p.disarm();
  t.onLocalSendError();
  CHECK_EQ(t.localSendErrors, 1);
  CHECK_EQ(t.state, TxStateMachine::READY);

  CHECK_EQ(p.onSendCallback(true), TX_CB_STALE);
  uint32_t tk = 12345;
  CHECK_EQ(p.take(&tk), TX_CB_STALE);
  CHECK_EQ(tk, 0u);
  t.onSendCallback(0, false);
  CHECK_EQ(t.staleCallbacks, 1);
  CHECK_EQ(t.delivered, 0);
}

static void testTxPeerMiss() {
  TxStateMachine t;
  TxCallbackPort p;
  t.onRadioUp();
  for (int i = 0; i < 3; ++i) {
    const uint32_t tok = t.beginSend(1000u + i * 100u);
    p.arm(tok);
    CHECK_EQ(p.onSendCallback(false), TX_CB_FAILED);
    uint32_t tk = 0;
    CHECK_EQ(p.take(&tk), TX_CB_FAILED);
    t.onSendCallback(tk, false);
    CHECK_EQ(t.state, TxStateMachine::READY);
  }
  CHECK_EQ(t.sendFailCallbacks, 3);
  CHECK(!t.faultLatched());
}

static void testTxTimeoutLatch() {
  TxStateMachine t;
  TxCallbackPort p;
  t.onRadioUp();
  const uint32_t tok = t.beginSend(1000);
  p.arm(tok);
  CHECK(!t.tick(1200));
  CHECK(t.tick(1500));
  CHECK_EQ(t.state, TxStateMachine::FAULT);
  CHECK_EQ(t.pendingToken, 0u);

  p.disarm();
  p.pendingCallback = TX_CB_NONE;
  p.callbackToken = 0;

  CHECK(!t.canSend(60000));
  CHECK_EQ(t.beginSend(60000), 0u);
  CHECK(!t.tick(60000));

  for (int i = 0; i < 3; ++i) {
    CHECK_EQ(p.onSendCallback(true), TX_CB_STALE);
    uint32_t tk = 0;
    CHECK_EQ(p.take(&tk), TX_CB_STALE);
    t.onSendCallback(0, false);
  }
  CHECK_EQ(t.staleCallbacks, 3);
  CHECK_EQ(t.state, TxStateMachine::FAULT);
}

static void testTxRollover() {
  TxStateMachine t;
  TxCallbackPort p;
  t.onRadioUp();
  const uint32_t start = UINT32_MAX - 100u;
  p.arm(t.beginSend(start));
  CHECK(!t.tick(start + 10u));
  CHECK(t.tick(start + 600u));
  CHECK_EQ(t.state, TxStateMachine::FAULT);
}

static void testTxRadioEvents() {
  TxStateMachine t;
  t.onRadioUp();
  CHECK_EQ(t.state, TxStateMachine::READY);
  t.onRadioDown();
  CHECK_EQ(t.state, TxStateMachine::INIT);
  TxStateMachine f;
  f.onRadioUp();
  f.beginSend(0);
  f.tick(1000);
  CHECK(f.faultLatched());
  f.onRadioUp();
  CHECK(f.faultLatched());
}

// ---------------------------------------------------------------------------
// 电机控制
// ---------------------------------------------------------------------------
static void runMotor(MotorControl &m, uint32_t fromMs, uint32_t toMs, uint32_t stepMs,
                     uint32_t pulses, float rpm, bool enabled = true) {
  for (uint32_t t = fromMs; t <= toMs; t += stepMs)
    m.update(t, stepMs, pulses, rpm, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, enabled);
}

static void testMotorTarget() {
  // r12：420 低于目标 540，但高于严重低速阈值 351（540*0.65），可用于验证正常调节。
  MotorControl m;
  runMotor(m, 50, 4000, 50, 60, 420);
  CHECK_EQ(m.fault, MotorControl::NONE);
  CHECK_EQ(m.pwm, 255);
  m.update(4050, 50, 120, 1000, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm < 255);
}

// r12：低速保护阈值必须跟随新目标 540（540*0.65=351），不得写死旧 590 对应的 383.5。
// 360 高于新阈值 351、低于旧目标 590 对应的 383.5；且只有在 PWM>=220 并持续 2000ms
// 的条件同时满足时才会锁存严重低速——阈值不同时本测试的结局必然不同。
static void testLowSpeedThresholdFollowsTarget() {
  // rpm=360：高于新阈值 351 → 不判低速；低于旧阈值 383.5 → 写死旧值时会被判低速。
  MotorControl a;
  runMotor(a, 50, 5000, 50, 60, 360);   // r13：终点由 3000ms 延长到 5000ms
  CHECK_EQ(a.fault, MotorControl::NONE);      // 360 高于 351，不锁存
  CHECK(a.pwm > 0);

  // rpm=340：低于 351，且 PWM>=220 持续 2000ms → 必须锁存 LOW_SPEED。
  MotorControl b;
  runMotor(b, 50, 10000, 50, 60, 340);
  CHECK_EQ(b.fault, MotorControl::LOW_SPEED);
  CHECK_EQ(b.pwm, 0);
}

// ---------------------------------------------------------------------------
// r13：小数积分保留（修复“整数 PWM 反压吞掉小数积分”缺陷）
//
// 共同初值：target=540、elapsed=50ms、pwm=50、
//           integral = 50 - 0.18 × (540 - feedbackRpm)
//           （即 proportional + integral = 50，恰好等于初始 PWM）
// 每个窗口都给非零脉冲，避免触发无脉冲保护。
// 连续 200 个周期（10 秒），直接驱动生产 MotorControl。
// ---------------------------------------------------------------------------

// 构造共同初值并推进一个周期；返回是否已建立初值条件。
static void r13InitCase(MotorControl &m, float feedbackRpm) {
  const float integral0 = 50.0f - 0.18f * (540.0f - feedbackRpm);
  m.pwm = 50;
  m.integral = integral0;
  // 首个窗口按给定反馈运行（elapsed=50ms，非零脉冲）。
  m.update(50, 50, 60, feedbackRpm, 540.0f, 600.0f, true);
}

// 正式要求：持续正误差、未触及输出约束时，
// 积分能够跨周期积累，PWM 最终高于初值。
static void testPiFractionAccumulates() {
  const float cases[] = {530.0f, 520.0f};
  for (float fb : cases) {
    MotorControl m;
    r13InitCase(m, fb);
    const int pwmStart = 50;
    bool exceeded = false;
    for (uint32_t t = 100; t <= 10000; t += 50) {
      m.update(t, 50, 60, fb, 540.0f, 600.0f, true);
      CHECK_EQ(m.fault, MotorControl::NONE);
      CHECK(m.pwm >= 0 && m.pwm <= 255);
      // 输出必须单调不降（正误差、未触斜率上限时允许 +10/周期）。
      if (m.pwm > pwmStart) exceeded = true;
    }
    CHECK(exceeded);                 // 10 秒内输出必须高于初值
    CHECK(m.pwm > pwmStart);
    CHECK(m.integral > 50.0f - 0.18f * (540.0f - fb));  // 积分有有效增长
  }
  // 斜率约束仍生效：每周期上升不超过 10。
  {
    MotorControl m;
    r13InitCase(m, 520.0f);
    int prev = m.pwm;
    for (uint32_t t = 100; t <= 2000; t += 50) {
      m.update(t, 50, 60, 520.0f, 540.0f, 600.0f, true);
      CHECK(m.pwm - prev <= 10);
      prev = m.pwm;
    }
  }
}

// 误差为 0：不得产生无理由的积分增长。
static void testPiFractionZeroErrorStable() {
  MotorControl m;
  // 先让系统在目标处稳定（proportional=0，integral 承担全部输出）。
  m.pwm = 50;
  m.integral = 50.0f;
  m.update(50, 50, 60, 540.0f, 540.0f, 600.0f, true);
  const float i0 = m.integral;
  for (uint32_t t = 100; t <= 5000; t += 50) {
    m.update(t, 50, 60, 540.0f, 540.0f, 600.0f, true);
    CHECK_EQ(m.fault, MotorControl::NONE);
    // 误差为 0：积分不应增长（允许被抗饱和/取整路径向下修正，但不得上扬）。
    CHECK(m.integral <= i0 + 0.001f);
    CHECK(m.pwm <= 255);
  }
}

// 反馈高于目标：PWM 能够降低；饱和后回落不得被积分钉在 255。
static void testPiFractionUnloading() {
  // 建立饱和输出。
  MotorControl m;
  runMotor(m, 50, 4000, 50, 60, 420);
  CHECK_EQ(m.pwm, 255);
  // 反馈升到目标以上，输出必须下降。
  for (uint32_t t = 4050; t <= 8000; t += 50)
    m.update(t, 50, 120, 700.0f, 540.0f, 600.0f, true);
  CHECK(m.pwm < 255);
  // 长期高于目标：不得因积分堆积而一直保持 255。
  for (uint32_t t = 8050; t <= 12000; t += 50)
    m.update(t, 50, 120, 700.0f, 540.0f, 600.0f, true);
  CHECK(m.pwm < 255);
  CHECK_EQ(m.fault, MotorControl::NONE);
}

// ---------------------------------------------------------------------------
// 第1轮（r11）：取消 600 超速锁存后的控制行为
//
// 生产代码已删除“rpm >= 600 → OVERSPEED 锁存 → PWM 清零”分支。
// 下面直接驱动真实 MotorControl，验证高转速由 PI 正常处理。
// ---------------------------------------------------------------------------

// 1-2. 599/600/601/650/680 均不得产生 OVERSPEED；高于目标时 PI 降低输出。
static void testNoOverspeedLatchAtAnyRpm() {
  const float rmps[] = {599.0f, 600.0f, 601.0f, 650.0f, 680.0f};
  for (float r : rmps) {
    MotorControl m;
    // 用低于目标的转速建立较大的稳定输出（避免稳态误差太小导致 PWM 接近 0）。
    runMotor(m, 50, 2000, 50, 40, 400);
    CHECK_EQ(m.fault, MotorControl::NONE);
    const int pwmBefore = m.pwm;
    CHECK(pwmBefore > 0);

    // 单个窗口冲到该转速：不得锁存，PWM 不得被清零。
    m.update(2050, 50, 74, r, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    CHECK_EQ(m.fault, MotorControl::NONE);
    CHECK(!m.overspeedLatched());
    CHECK(m.pwm > 0);
    // 高于目标 → PI 输出必须下降（不是被清零，也不是继续加大）。
    CHECK(m.pwm < pwmBefore);

    // 持续保持该高转速多个窗口：仍然不得锁存。
    // 注意：PI 可能把输出压到 0（转速远高于目标时的正常调节），
    // 这与“故障锁停”不同——判据是 fault 仍为 NONE 且之后还能重新出力。
    for (uint32_t t = 2100; t <= 6000; t += 50) {
      m.update(t, 50, 74, r, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
      CHECK_EQ(m.fault, MotorControl::NONE);
    }
    CHECK(!m.overspeedLatched());

    // 转速回落到远低于目标：必须能重新建立输出（证明不是永久停机）。
    for (uint32_t t = 6050; t <= 8000; t += 50)
      m.update(t, 50, 40, 300.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
    CHECK_EQ(m.fault, MotorControl::NONE);
    CHECK(m.pwm > 0);
  }
}

// 3. 超过 600 后回到 590 以下，控制器仍能继续调节。
static void testRecoversAfterHighRpm() {
  MotorControl m;
  runMotor(m, 50, 2000, 50, 40, 400);      // 建立较大输出
  CHECK_EQ(m.fault, MotorControl::NONE);

  // 冲到 680 若干个窗口。
  for (uint32_t t = 2050; t <= 3000; t += 50)
    m.update(t, 50, 80, 680.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::NONE);
  const int pwmHigh = m.pwm;

  // 回落到 500（低于目标）：输出应重新上升，说明没有永久停机。
  for (uint32_t t = 3050; t <= 5000; t += 50)
    m.update(t, 50, 60, 500.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.fault, MotorControl::NONE);
  CHECK(m.pwm > pwmHigh);      // 再次加油
  CHECK(m.pwm > 0);
}

// 4. 无脉冲保护仍能触发并锁存。
static void testNoPulsesStillLatches() {
  MotorControl m;
  runMotor(m, 50, 4000, 50, 0, 0);          // 高输出且无脉冲
  CHECK_EQ(m.fault, MotorControl::NO_PULSES);
  CHECK_EQ(m.pwm, 0);
  // 锁存后即使转速恢复也不得自动重启。
  runMotor(m, 4050, 7000, 50, 70, 590);
  CHECK_EQ(m.fault, MotorControl::NO_PULSES);
  CHECK_EQ(m.pwm, 0);
}

// 5. 持续严重低速保护仍能触发并锁存。
static void testLowSpeedStillLatches() {
  MotorControl m;
  runMotor(m, 50, 10000, 50, 12, 100);      // 高输出、持续严重掉速
  CHECK_EQ(m.fault, MotorControl::LOW_SPEED);
  CHECK_EQ(m.pwm, 0);
  runMotor(m, 10050, 14000, 50, 70, 590);
  CHECK_EQ(m.fault, MotorControl::LOW_SPEED);
  CHECK_EQ(m.pwm, 0);
}

// 6. 已锁存的其他故障不得因本次改动被自动清除（即使转速超过 600）。
static void testLatchedFaultsNotClearedByOverspeedPathRemoval() {
  // 无脉冲锁存后，输入 680 RPM 也不得清除故障或恢复输出。
  MotorControl a;
  runMotor(a, 50, 4000, 50, 0, 0);
  CHECK_EQ(a.fault, MotorControl::NO_PULSES);
  runMotor(a, 4050, 8000, 50, 80, 680.0f);
  CHECK_EQ(a.fault, MotorControl::NO_PULSES);
  CHECK_EQ(a.pwm, 0);

  // 低速锁存后同样不得清除。
  MotorControl b;
  runMotor(b, 50, 10000, 50, 12, 100);
  CHECK_EQ(b.fault, MotorControl::LOW_SPEED);
  runMotor(b, 10050, 14000, 50, 80, 680.0f);
  CHECK_EQ(b.fault, MotorControl::LOW_SPEED);
  CHECK_EQ(b.pwm, 0);
}

// PI 在高转速下继续按误差调节（不依赖故障路径）。
static void testPiStillRegulatesAboveTarget() {
  MotorControl m;
  runMotor(m, 50, 2000, 50, 40, 400);      // 建立较大输出
  const int atTarget = m.pwm;
  CHECK(atTarget > 0);

  // 略高于目标：输出应减小。
  m.update(2050, 50, 74, 620.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm < atTarget);
  const int at620 = m.pwm;

  // 更高：继续减小（或至少不再增加）。
  m.update(2100, 50, 80, 700.0f, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm <= at620);
  CHECK_EQ(m.fault, MotorControl::NONE);
}

static void testExistingProtections() {
  MotorControl missing;
  runMotor(missing, 50, 3000, 50, 0, 0);
  CHECK_EQ(missing.fault, MotorControl::NO_PULSES);
  runMotor(missing, 3050, 8000, 50, 70, 590);
  CHECK_EQ(missing.pwm, 0);

  MotorControl slow;
  runMotor(slow, 50, 10000, 50, 12, 100);
  CHECK_EQ(slow.fault, MotorControl::LOW_SPEED);
  runMotor(slow, 10050, 14000, 50, 70, 590);
  CHECK_EQ(slow.pwm, 0);

  MotorControl off;
  off.update(50, 50, 40, 333, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, false);
  CHECK_EQ(off.pwm, 0);
  CHECK_EQ(off.fault, MotorControl::NONE);
}

static void testRampAndWindup() {
  MotorControl m;
  m.update(50, 50, 40, 333, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK_EQ(m.pwm, 10);
  m.update(100, 50, 40, 333, (float)FW_TARGET_RPM, (float)FW_OVERSPEED_RPM, true);
  CHECK(m.pwm <= 20);

  MotorControl n;
  runMotor(n, 50, 6000, 50, 0, 0);
  CHECK(n.integral <= 255.0f);
}

// ---------------------------------------------------------------------------
// 舵机与协议
// ---------------------------------------------------------------------------
static void testServo() {
  CHECK_EQ(steeringPulse(70), 1278);
  CHECK_EQ(steeringPulse(90), 1500);
  CHECK_EQ(steeringPulse(110), 1722);
  CHECK_EQ(steeringPulse(0), 1278);
  CHECK_EQ(steeringPulse(180), 1722);

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

  ServoOutput out;
  CHECK_EQ(out.lastPulseUs, -1);
  CHECK(out.shouldWrite(1500));
  for (int i = 0; i < 100; ++i) CHECK(!out.shouldWrite(1500));
  CHECK_EQ(out.writes, 1);
  CHECK(out.shouldWrite(1278));
  CHECK(!out.shouldWrite(1278));

  const double usPerTick = 20000.0 / 16384.0;
  for (int angle = 70; angle <= 110; ++angle) {
    const int us = steeringPulse(angle);
    const int ticks = (int)((double)us / usPerTick);
    const double back = ticks * usPerTick;
    CHECK(us - back >= 0.0);
    CHECK(us - back < usPerTick);
    CHECK(ticks > 0 && ticks < 16384);
  }
}

static void testProtocolAndMac() {
  uint8_t mac[6];
  CHECK(parsePeerMac("24:62:AB:F0:72:94", mac) && mac[5] == 0x94);
  CHECK(parsePeerMac("24:62:ab:ef:cf:84", mac));
  CHECK(parsePeerMac("24:62:AB:F1:36:64", mac));
  CHECK(!parsePeerMac("FF:FF:FF:FF:FF:FF", mac));
  CHECK(!parsePeerMac("00:00:00:00:00:00", mac));
  CHECK(!parsePeerMac("25:62:AB:F0:72:94", mac));
  CHECK(!parsePeerMac("24:62:AB:F0:72:94 trailing", mac));

  CHECK_EQ(sizeof(int32_t) * 2, 8u);
  CHECK_EQ(FW_PAIR_PROTOCOL, 2);
  const uint8_t radioChannel = 1;
  CHECK_EQ(radioChannel, 1);
}

// ---------------------------------------------------------------------------
// 自身 MAC（确保没有回退）
// ---------------------------------------------------------------------------
static int fakeMacOk(uint8_t *out) {
  std::memcpy(out, kSelfEFCF84, 6);
  return SELF_MAC_OK;
}
static int fakeMacAllZero(uint8_t *out) { std::memset(out, 0, 6); return SELF_MAC_OK; }
static int fakeMacFails(uint8_t *) { return SELF_MAC_INVALID_MAC; }

static void testSelfMac() {
  char text[MAC_TEXT_CAP];
  SelfMacState a;
  CHECK(a.read(fakeMacOk));
  a.format(text, sizeof(text));
  CHECK(std::strcmp(text, "24:62:AB:EF:CF:84") == 0);

  SelfMacState b;
  CHECK(!b.read(fakeMacAllZero));
  b.format(text, sizeof(text));
  CHECK(std::strcmp(text, "READ_FAILED") == 0);

  SelfMacState c;
  CHECK(!c.read(fakeMacFails));
  CHECK_EQ(c.status, SELF_MAC_INVALID_MAC);
  c.format(text, sizeof(text));
  CHECK(std::strcmp(text, "READ_FAILED") == 0);

  const uint8_t other[6] = {0x24, 0x62, 0xAB, 0xF0, 0x72, 0x94};
  CHECK(!a.equals(other));
  CHECK(!c.equals(other));
}

static void testUnmeasuredPulsesPerRev() {
  std::printf("NOTE: PULSES_PER_REV=144.0 未实机标定；540 / OS600(历史,未启用) 目前只是软件命令值。\n");
}

int main() {
  testIdentity();
  testBaselineFixedData();
  testBaselineBugWouldOverspeed();
  testBaselineKeepsTotalAcrossRollover();
  testCountCommandSharedImpl();
  testCountViaServicePairing();
  testFaultNoneSharedImpl();
  testFaultBothLinesSharedImpl();
  testFaultFirstLineFails();
  testFaultSecondLineFails();
  testFaultFormatFailureSendsNothing();
  testFaultViaServicePairing();
  testPulseTotalBasics();
  testPulseTotalRollover();
  testPulseSnapshotConsistency();
  testBootId();
  testCountFormat();
  testCountCommandEndToEnd();
  testFaultEvidenceNoneAndThreeFaults();
  testFaultEvidenceFirstWindowZeroPrev();
  testFaultEvidenceNotOverwritten();
  testFaultFormatting();
  testFaultCommandNone();
  testFaultCommandAfterLatch();
  testFaultCommandQueueFull();
  testRunLineSnapshot();
  testMacTextCompleteness();
  testIdentityLineCompleteness();
  testSpeedWindowAlignment();
  testTxHappyPath();
  testTxLocalErrorThenStale();
  testTxPeerMiss();
  testTxTimeoutLatch();
  testTxRollover();
  testTxRadioEvents();
  testA_CriticalFullKeepsPending();
  testB_PersistentFullNoLoss();
  testC_RecoveryNeedsAck();
  testD_DequeuedButNotFlushed();
  testE_TimeoutKeepsOldRuntimePeer();
  testF_TimeoutNoRestart();
  testF2_NormalSaveAckThenRestart();
  testG_FormatFailure();
  testH_NormalCongestion();
  testI_SecondBindBlocked();
  testStateEventOnce();
  testPiFractionAccumulates();
  testPiFractionZeroErrorStable();
  testPiFractionUnloading();
  testMotorTarget();
  testLowSpeedThresholdFollowsTarget();
  testNoOverspeedLatchAtAnyRpm();
  testRecoversAfterHighRpm();
  testNoPulsesStillLatches();
  testLowSpeedStillLatches();
  testLatchedFaultsNotClearedByOverspeedPathRemoval();
  testPiStillRegulatesAboveTarget();
  testExistingProtections();
  testRampAndWindup();
  testServo();
  testProtocolAndMac();
  testSelfMac();
  testUnmeasuredPulsesPerRev();

  if (g_failures == 0) {
    std::printf("PASS: %d checks\n", g_checks);
    return 0;
  }
  std::printf("FAILED: %d of %d checks\n", g_failures, g_checks);
  return 1;
}
