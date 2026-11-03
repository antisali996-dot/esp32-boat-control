#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "fw_role.h"
#include "pair_mac.h"

// 身份行（v3）以 JAYv3 开头并带 FWID=<真实固件身份串>。
// 同时保留一行旧格式 JAY ROLE=... VERSION=2 ...，让尚未更新的旧工具仍能读到角色和 MAC。
// 工具的判定规则：只有 FWID 与本包固件清单完全一致才算最新；
// 旧固件没有 FWID（或 FWID 不同）一律提示升级，配对协议版本不参与该判断。
inline void loadPair(uint8_t *peer) {
  Preferences prefs;
  if (!prefs.begin("jay-pair", false)) return;
  uint8_t saved[6];
  if (prefs.getBytesLength("peer") == 6 && prefs.getBytes("peer", saved, 6) == 6 && validPeerMac(saved))
    memcpy(peer, saved, 6);
  prefs.end();
}
inline void pairInfo(const char *role, const uint8_t *peer) {
  Serial.printf("JAYv3 ROLE=%s MAC=%s PEER=%02X:%02X:%02X:%02X:%02X:%02X PAIR=%d STEER=%d FWID=%s VER=%s\n",
    role, WiFi.macAddress().c_str(), peer[0],peer[1],peer[2],peer[3],peer[4],peer[5],
    FW_PAIR_PROTOCOL, FW_STEER_DEG, FW_ID, FW_VER);
  // 兼容尚未升级的旧向导：STEER= 与实际控制一致，可安全沿用。
  Serial.printf("JAY ROLE=%s MAC=%s PEER=%02X:%02X:%02X:%02X:%02X:%02X VERSION=%d STEER=%d\n",
    role, WiFi.macAddress().c_str(), peer[0],peer[1],peer[2],peer[3],peer[4],peer[5],
    FW_PAIR_PROTOCOL, FW_STEER_DEG);
}
// USB 本地配置，不经无线修改。断开动力后使用；保存成功才重启。
inline void servicePairing(const char *role, const uint8_t *peer) {
  static String line;
  static bool overflow = false;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (line.length() < 80 && !overflow) line += c;
      else overflow = true;
      continue;
    }
    if (overflow) { Serial.println("JAY ERROR=LINE_TOO_LONG"); }
    else if (line == "JAY INFO") pairInfo(role, peer);
    else if (line == "JAY PING") Serial.printf("JAY PONG FWID=%s VER=%s\n", FW_ID, FW_VER);
    else if (line.startsWith("JAY BIND ")) {
      uint8_t replacement[6], own[6];
      WiFi.macAddress(own);
      if (!parsePeerMac(line.substring(9).c_str(), replacement) || memcmp(own,replacement,6)==0) {
        Serial.println("JAY ERROR=INVALID_PEER");
      } else {
        Preferences prefs;
        bool saved = prefs.begin("jay-pair", false);
        if (saved) { saved = prefs.putBytes("peer", replacement, 6) == 6; prefs.end(); }
        if (!saved) Serial.println("JAY ERROR=SAVE_FAILED");
        else { Serial.println("JAY SAVED REBOOT"); Serial.flush(); delay(100); ESP.restart(); }
      }
    } else {
      Serial.println("JAY ERROR=UNKNOWN_COMMAND");
    }
    line = "";
    overflow = false;
  }
}
