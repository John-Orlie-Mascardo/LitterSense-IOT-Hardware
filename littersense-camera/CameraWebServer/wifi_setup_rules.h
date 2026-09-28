#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct WifiCredentials {
  char ssid[33];
  char password[65];
};

inline int setupHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Decode one form field, rejecting duplicates, truncation and embedded NULs.
inline bool setupField(const char *body, const char *name, char *out, size_t capacity) {
  bool found = false;
  for (const char *part = body; *part;) {
    const char *end = strchr(part, '&');
    if (!end) end = part + strlen(part);
    const char *eq = static_cast<const char *>(memchr(part, '=', end - part));
    if (eq && size_t(eq - part) == strlen(name) && !strncmp(part, name, eq - part)) {
      if (found) return false;
      found = true;
      size_t n = 0;
      for (const char *p = eq + 1; p < end; ++p) {
        unsigned char c = *p;
        if (c == '+') c = ' ';
        else if (c == '%') {
          if (end - p < 3 || setupHex(p[1]) < 0 || setupHex(p[2]) < 0) return false;
          c = (setupHex(p[1]) << 4) | setupHex(p[2]);
          p += 2;
        }
        if (!c || n + 1 >= capacity) return false;
        out[n++] = c;
      }
      out[n] = 0;
    }
    part = *end ? end + 1 : end;
  }
  return found;
}

inline bool validWifiCredentials(const WifiCredentials &c) {
  if (!memchr(c.ssid, 0, sizeof(c.ssid)) || !memchr(c.password, 0, sizeof(c.password))) return false;
  const size_t ssid = strlen(c.ssid);
  const size_t password = strlen(c.password);
  if (!ssid || ssid > 32 || password > 64) return false;
  if (password && password < 8) return false;
  for (size_t i = 0; i < password; ++i) {
    if (password == 64 ? setupHex(c.password[i]) < 0 :
        (static_cast<unsigned char>(c.password[i]) < 32 || static_cast<unsigned char>(c.password[i]) > 126)) return false;
  }
  return true;
}

inline bool setupElapsed(uint32_t now, uint32_t since, uint32_t duration) {
  return uint32_t(now - since) >= duration;
}

inline bool setupNetworksOverlap(uint32_t a, uint32_t aMask, uint32_t b, uint32_t bMask) {
  return (a & (aMask & bMask)) == (b & (aMask & bMask));
}
