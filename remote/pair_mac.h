#pragma once
#include <stdint.h>
#include <cstring>
#include <cctype>
#include <cstdlib>
inline bool validPeerMac(const uint8_t *mac) {
  bool nonzero = false;
  for (int i = 0; i < 6; ++i) nonzero |= mac[i] != 0;
  return nonzero && (mac[0] & 1) == 0;
}
inline bool parsePeerMac(const char *text, uint8_t *mac) {
  if (!text || strlen(text) != 17) return false;
  for (int i = 0; i < 17; ++i) {
    if (i % 3 == 2) { if (text[i] != ':') return false; }
    else if (!isxdigit((unsigned char)text[i])) return false;
  }
  for (int i = 0; i < 6; ++i) { char byte[3]={text[i*3],text[i*3+1],0};mac[i]=strtoul(byte,nullptr,16); }
  return validPeerMac(mac);
}
