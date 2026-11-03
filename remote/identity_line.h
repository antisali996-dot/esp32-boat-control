#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>

// 身份行与 MAC 文本的共用格式化逻辑（纯逻辑，不依赖 Arduino）。
//
// 固件的 pairing.h 与电脑测试都调用这里，因此测试覆盖的是固件真正使用的
// 格式字符串，不存在“测试另抄一份格式”导致两边漂移的可能。
//
// 所有格式化都检查 snprintf 的返回值：
//   返回负数  → 编码错误（LOG_FMT_ERROR）；
//   返回值 >= 容量 → 已被截断（LOG_FMT_TRUNCATED），不得当作合法协议行发送。

constexpr size_t MAC_TEXT_CAP = 18;        // "XX:XX:XX:XX:XX:XX" = 17 + NUL
constexpr size_t IDENTITY_LINE_CAP = 256;  // 身份行与普通日志的统一容量

enum FmtStatus { FMT_OK = 0, FMT_ERROR = 1, FMT_TRUNCATED = 2 };

inline FmtStatus checkFmt(int written, size_t cap) {
  if (written < 0) return FMT_ERROR;
  if ((size_t)written >= cap) return FMT_TRUNCATED;
  return FMT_OK;
}

// 把 MAC 写成 XX:XX:XX:XX:XX:XX。
// 读取失败时写 "READ_FAILED"，绝不输出 00:00:00:00:00:00 冒充合法地址。
inline FmtStatus formatMacText(const uint8_t *mac, bool valid, char *out, size_t cap) {
  if (!valid) {
    const int n = std::snprintf(out, cap, "READ_FAILED");
    return checkFmt(n, cap);
  }
  const int n = std::snprintf(out, cap, "%02X:%02X:%02X:%02X:%02X:%02X",
                              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return checkFmt(n, cap);
}

// 新版身份行（带 FWID）。字段名与顺序与工具端约定一致，不得删减。
inline FmtStatus formatIdentityLine(char *out, size_t cap, const char *role,
                                    const char *macText, const uint8_t *peer,
                                    int pairProtocol, int steerDeg,
                                    const char *fwid, const char *ver) {
  const int n = std::snprintf(
      out, cap,
      "JAYv4 ROLE=%s MAC=%s PEER=%02X:%02X:%02X:%02X:%02X:%02X PAIR=%d STEER=%d FWID=%s VER=%s",
      role, macText, peer[0], peer[1], peer[2], peer[3], peer[4], peer[5],
      pairProtocol, steerDeg, fwid, ver);
  return checkFmt(n, cap);
}

// 兼容旧工具的旧格式行（无 FWID）。
inline FmtStatus formatLegacyLine(char *out, size_t cap, const char *role,
                                  const char *macText, const uint8_t *peer,
                                  int pairProtocol, int steerDeg) {
  const int n = std::snprintf(
      out, cap,
      "JAY ROLE=%s MAC=%s PEER=%02X:%02X:%02X:%02X:%02X:%02X VERSION=%d STEER=%d",
      role, macText, peer[0], peer[1], peer[2], peer[3], peer[4], peer[5],
      pairProtocol, steerDeg);
  return checkFmt(n, cap);
}

// 自身 MAC 读取失败时的身份行：只剩角色、协议与 FWID，明确标注 MAC 不可用。
inline FmtStatus formatIdentityLineNoMac(char *out, size_t cap, const char *role,
                                         int pairProtocol, int steerDeg,
                                         const char *fwid, const char *ver) {
  const int n = std::snprintf(out, cap, "JAYv4 ROLE=%s MAC=READ_FAILED PAIR=%d STEER=%d FWID=%s VER=%s",
                              role, pairProtocol, steerDeg, fwid, ver);
  return checkFmt(n, cap);
}

// 一次身份回应的完整结果。固件的 postIdentity() 与电脑测试都走这里，
// 因此测试覆盖的正是工具端会收到的那两条格式串。
struct IdentityLines {
  char main[IDENTITY_LINE_CAP];     // JAYv4 行（或读取失败行）
  char legacy[IDENTITY_LINE_CAP];   // 旧工具兼容行；读取失败时为空
  FmtStatus mainStatus = FMT_ERROR;
  FmtStatus legacyStatus = FMT_ERROR;
  bool selfMacValid = false;
  char macText[MAC_TEXT_CAP];

  bool mainOk() const { return mainStatus == FMT_OK; }
  bool legacyOk() const { return legacyStatus == FMT_OK; }
};

inline IdentityLines buildIdentityLines(const char *role, const uint8_t *selfMac, bool selfOk,
                                        const uint8_t *peer, int pairProtocol, int steerDeg,
                                        const char *fwid, const char *ver) {
  IdentityLines r;
  r.selfMacValid = selfOk;
  r.mainStatus = formatMacText(selfMac, selfOk, r.macText, sizeof(r.macText));
  if (r.mainStatus != FMT_OK) return r;

  if (!selfOk) {
    // 读取失败：只输出明确的失败行，不伪造全零 MAC，也不输出旧格式行。
    r.mainStatus = formatIdentityLineNoMac(r.main, sizeof(r.main), role, pairProtocol,
                                           steerDeg, fwid, ver);
    r.legacy[0] = '\0';
    r.legacyStatus = FMT_ERROR;
    return r;
  }

  r.mainStatus = formatIdentityLine(r.main, sizeof(r.main), role, r.macText, peer,
                                    pairProtocol, steerDeg, fwid, ver);
  r.legacyStatus = formatLegacyLine(r.legacy, sizeof(r.legacy), role, r.macText, peer,
                                    pairProtocol, steerDeg);
  return r;
}
