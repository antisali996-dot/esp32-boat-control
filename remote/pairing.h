#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "pair_mac.h"

inline void loadPair(uint8_t *peer) {
  Preferences prefs;
  if (!prefs.begin("jay-pair", false)) return;
  uint8_t saved[6];
  if (prefs.getBytesLength("peer") == 6 && prefs.getBytes("peer", saved, 6) == 6 && validPeerMac(saved))
    memcpy(peer, saved, 6);
  prefs.end();
}
inline void pairInfo(const char *role, const uint8_t *peer) {
  Serial.printf("JAY ROLE=%s MAC=%s PEER=%02X:%02X:%02X:%02X:%02X:%02X VERSION=2 STEER=20\n",
    role, WiFi.macAddress().c_str(), peer[0],peer[1],peer[2],peer[3],peer[4],peer[5]);
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
    }
    line = "";
    overflow = false;
  }
}
