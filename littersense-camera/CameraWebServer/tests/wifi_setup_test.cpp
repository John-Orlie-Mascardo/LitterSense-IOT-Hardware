#include "../wifi_setup_rules.h"
#include <assert.h>
#include <stdio.h>

int main() {
  WifiCredentials c = {};
  assert(setupField("ssid=Cafe+%26+Cats&password=+password+", "ssid", c.ssid, sizeof(c.ssid)));
  assert(!strcmp(c.ssid, "Cafe & Cats"));
  assert(setupField("password=+password+", "password", c.password, sizeof(c.password)));
  assert(!strcmp(c.password, " password ")); // Spaces are credentials, never trim.
  assert(validWifiCredentials(c));
  assert(!setupField("ssid=one&ssid=two", "ssid", c.ssid, sizeof(c.ssid)));
  assert(!setupField("ssid=%00evil", "ssid", c.ssid, sizeof(c.ssid)));
  assert(!setupField("ssid=%xy", "ssid", c.ssid, sizeof(c.ssid)));
  assert(!setupField("ssid=%2", "ssid", c.ssid, sizeof(c.ssid)));
  assert(!setupField("ssid=123456789012345678901234567890123", "ssid", c.ssid, sizeof(c.ssid)));
  assert(setupField("ssid=%E7%8C%AB", "ssid", c.ssid, sizeof(c.ssid)));
  assert(setupField("password=", "password", c.password, sizeof(c.password)));
  assert(validWifiCredentials(c)); // Open networks supported locally.
  strcpy(c.password, "short"); assert(!validWifiCredentials(c));
  memset(c.password, 'a', 64); c.password[64] = 0; assert(validWifiCredentials(c));
  c.password[5] = 'z'; assert(!validWifiCredentials(c));
  memset(c.ssid, 'x', sizeof(c.ssid)); assert(!validWifiCredentials(c));
  assert(!setupElapsed(29999, 0, 30000));
  assert(setupElapsed(30000, 0, 30000));
  assert(setupElapsed(20, UINT32_MAX - 20, 40));
  assert(setupNetworksOverlap(0xc0a80480, 0xffffff00, 0xc0a80401, 0xffffff00));
  assert(setupNetworksOverlap(0xc0a80801, 0xffff0000, 0xc0a80401, 0xffffff00));
  assert(!setupNetworksOverlap(0xc0a80480, 0xffffff00, 0x0a2a0001, 0xffffff00));
  puts("PASS: form decoding, credential validation, timeout rollover and subnet overlap");
}
