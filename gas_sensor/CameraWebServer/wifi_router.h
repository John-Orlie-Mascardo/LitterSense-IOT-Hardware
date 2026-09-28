#pragma once

#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_netif.h>
#include <sdkconfig.h>

#if !CONFIG_LWIP_IP_FORWARD || !CONFIG_LWIP_IPV4_NAPT
#error "This ESP32 package lacks IP forwarding/NAPT. A hotspot alone cannot share internet."
#endif

// Set a private 8-63 character password before uploading. Never logged.
static const char *hotspotPassword = "1234567889";
static const IPAddress hotspotIP(192, 168, 4, 1);
static const IPAddress hotspotMask(255, 255, 255, 0);
// DHCP advertises this real DNS server; the S3 is not a DNS proxy.
// Use your home router's reachable DNS address if public DNS is blocked.
static const IPAddress hotspotDNS(1, 1, 1, 1);
static bool hotspotReady = false;
static bool natEnabled = false;
static IPAddress natHomeIP;

static bool setRouterNAT(bool enable) {
  esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  esp_err_t err = ap ? (enable ? esp_netif_napt_enable(ap) : esp_netif_napt_disable(ap)) : ESP_ERR_INVALID_STATE;
  if (err != ESP_OK) {
    Serial.printf("NAT ERROR %s: %s (0x%x)\n", enable ? "enable" : "disable", esp_err_to_name(err), err);
    return false;
  }
  natEnabled = enable;
  Serial.printf("NAPT %s; client internet still requires the phone test.\n", enable ? "enabled" : "disabled");
  return true;
}

static void startRouterWiFi(const char *homeSSID, const char *homePassword) {
  Serial.printf("Arduino ESP32 package: %s\n", ESP_ARDUINO_VERSION_STR);
  if (!WiFi.mode(WIFI_AP_STA)) {
    Serial.println("WiFi ERROR: AP+STA mode failed");
    return;
  }
  WiFi.setAutoReconnect(true);
  WiFi.begin(homeSSID, homePassword);
  WiFi.setSleep(false);
  Serial.println("Home WiFi connecting (2.4 GHz)...");
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 20000) delay(100);
  Serial.println(WiFi.status() == WL_CONNECTED ? "Home WiFi connected" : "Home WiFi unavailable; retrying in background");

  const size_t length = strlen(hotspotPassword);
  if (length < 8 || length > 63 || strcmp(hotspotPassword, "CHANGE_ME_LitterSense") == 0) {
    Serial.println("AP ERROR: set hotspotPassword to a private 8-63 character password");
    return;
  }
  // Four slots allow the two sensor boards and a phone to connect together.
  hotspotReady = WiFi.AP.begin()
    && WiFi.AP.config(hotspotIP, hotspotIP, hotspotMask, IPAddress(192, 168, 4, 2), hotspotDNS)
    && WiFi.AP.create("LitterSense", hotspotPassword, 1, 0, 4)
    && WiFi.AP.waitStatusBits(ESP_NETIF_STARTED_BIT, 1000);
  Serial.println(hotspotReady ? "LitterSense AP + DHCP started" : "AP/DHCP ERROR: startup failed");
}

static void serviceRouterWiFi() {
  static uint32_t lastReport = 0;
  static uint32_t lastReconnect = 0;
  const uint32_t now = millis();
  const bool connected = WiFi.status() == WL_CONNECTED && uint32_t(WiFi.localIP()) != 0;
  const IPAddress homeIP = WiFi.localIP();
  // Reject overlapping networks, including home networks wider than /24.
  const uint32_t commonMask = uint32_t(WiFi.subnetMask()) & uint32_t(hotspotMask);
  const bool overlap = connected && ((uint32_t(homeIP) & commonMask) == (uint32_t(hotspotIP) & commonMask));
  if (natEnabled && (!connected || overlap || homeIP != natHomeIP)) setRouterNAT(false);
  if (!connected && now - lastReconnect >= 30000) {
    lastReconnect = now;
    WiFi.reconnect();
  }
  if (now - lastReport < 5000) return;
  lastReport = now;
  if (hotspotReady && connected && !overlap && !natEnabled) {
    if (setRouterNAT(true)) natHomeIP = homeIP;
  }
  if (overlap) Serial.println("NAT ERROR: home/AP subnets overlap; change hotspotIP and DHCP start together");
  Serial.printf("Home: %s (status=%d), IP=%s | AP: %s IP=%s clients=%u | NAPT=%s | DHCP DNS=%s\n",
    connected ? "connected" : "disconnected", int(WiFi.status()), homeIP.toString().c_str(),
    hotspotReady ? "ready" : "failed", WiFi.softAPIP().toString().c_str(),
    WiFi.softAPgetStationNum(), natEnabled ? "enabled" : "disabled", hotspotDNS.toString().c_str());
}
