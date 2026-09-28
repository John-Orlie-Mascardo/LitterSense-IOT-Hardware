#pragma once

#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_netif.h>
#include <sdkconfig.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <esp_http_server.h>
#include <NetworkClient.h>
#include <atomic>
#include "wifi_setup_rules.h"
#include "wifi_setup_page.h"

#if !CONFIG_LWIP_IP_FORWARD || !CONFIG_LWIP_IPV4_NAPT
#error "This ESP32 package lacks IP forwarding/NAPT. A hotspot alone cannot share internet."
#endif

// Set a private 8-63 character password before uploading. Never logged.
static const char *hotspotPassword = "1234567889";
static IPAddress hotspotIP(192, 168, 4, 1);
static const IPAddress hotspotMask(255, 255, 255, 0);
// Normal operation advertises the destination router's DNS via DHCP.
static IPAddress hotspotDNS(1, 1, 1, 1);
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

static DNSServer setupDns;
static Preferences wifiPrefs;
static WifiCredentials savedWifi = {}, pendingWifi = {};
static QueueHandle_t setupQueue;
static bool storageReady = false, testingWifi = false;
static std::atomic<bool> setupActive{false};
enum class SetupState { Idle, Connecting, Connected, Failed, StorageError };
static std::atomic<SetupState> setupState{SetupState::Idle};
static char setupCsrf[33];
static uint32_t offlineSince, attemptSince, connectedSince, lastReconnect;
static bool connectionObserved = false;
static const uint32_t WIFI_RETRY_MS = 30000;
static const uint32_t WIFI_STABLE_MS = 3000;
static const uint32_t SETUP_SUCCESS_MS = 15000;

// Setup writes are reachable only through the setup AP, with a page-issued token.
static bool isSetupRequest(httpd_req_t *req) {
  // HTTPD uses dual-stack sockets; the core also handles IPv4-mapped IPv6.
  return setupActive && NetworkClient().localIP(httpd_req_to_sockfd(req)) == IPAddress(192, 168, 4, 1);
}

static bool isSetupHost(httpd_req_t *req) {
  char host[32];
  return httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) == ESP_OK
    && (!strcmp(host, "192.168.4.1") || !strcmp(host, "192.168.4.1:80"));
}

esp_err_t routerSetupPage(httpd_req_t *req) {
  if (!isSetupRequest(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Join LitterSense-Setup first.");
  if (!isSetupHost(req)) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, "", 0);
  }
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  httpd_resp_set_hdr(req, "Referrer-Policy", "no-referrer");
  String html(setupPage);
  html.replace("{{CSRF}}", setupCsrf);
  return httpd_resp_send(req, html.c_str(), html.length());
}

bool routerServeSetupRoot(httpd_req_t *req) {
  if (!isSetupRequest(req)) return false;
  routerSetupPage(req);
  return true;
}

static esp_err_t setupNotFound(httpd_req_t *req, httpd_err_code_t error) {
  if (!isSetupRequest(req)) return httpd_resp_send_err(req, error, "Not found");
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
  return httpd_resp_send(req, "", 0);
}

static esp_err_t setupStatus(httpd_req_t *req) {
  if (!isSetupRequest(req) || !isSetupHost(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Join setup Wi-Fi first.");
  const char *states[] = {"idle", "connecting", "connected", "failed", "storage_error"};
  char json[64];
  snprintf(json, sizeof(json), "{\"state\":\"%s\"}", states[static_cast<int>(setupState.load())]);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, json);
}

static esp_err_t provisionWifi(httpd_req_t *req) {
  if (!isSetupRequest(req) || !isSetupHost(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Open http://192.168.4.1 on setup Wi-Fi.");
  if (!setupQueue || !storageReady) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Setup storage unavailable. Restart the device.");
  if (req->content_len == 0 || req->content_len > 512) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid setup request size.");
  char body[513] = {}, csrf[33] = {};
  size_t received = 0;
  while (received < req->content_len) {
    int n = httpd_req_recv(req, body + received, req->content_len - received);
    if (n <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete setup request.");
    received += n;
  }
  if (memchr(body, 0, received) || !setupField(body, "csrf", csrf, sizeof(csrf)) || strcmp(csrf, setupCsrf))
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Open the device setup page and submit its form.");
  WifiCredentials next = {};
  if (!setupField(body, "ssid", next.ssid, sizeof(next.ssid)) || !setupField(body, "password", next.password, sizeof(next.password)) || !validWifiCredentials(next))
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use a 1-32 byte Wi-Fi name and an 8-63 character password (or blank for open Wi-Fi).");
  const auto state = setupState.load();
  if (state == SetupState::Connecting || state == SetupState::Connected)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "A connection attempt is already in progress.");
  setupState = SetupState::Connecting;
  if (xQueueSend(setupQueue, &next, 0) != pdTRUE) {
    setupState = state;
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Setup busy. Retry shortly.");
  }
  httpd_resp_set_status(req, "202 Accepted");
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_sendstr(req, "<!doctype html><meta http-equiv='refresh' content='2;url=/'><p>Testing Wi-Fi. Return to the setup page for the result.</p>");
}

void registerRouterSetup(httpd_handle_t server) {
  httpd_uri_t page = {};
  page.uri = "/setup"; page.method = HTTP_GET; page.handler = routerSetupPage;
  httpd_register_uri_handler(server, &page);
  page.uri = "/setup/status"; page.handler = setupStatus;
  httpd_register_uri_handler(server, &page);
  page.uri = "/provision"; page.method = HTTP_POST; page.handler = provisionWifi;
  httpd_register_uri_handler(server, &page);
  httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, setupNotFound);
}

static bool configureHotspot(bool setup) {
  setupActive = false;
  if (natEnabled && !setRouterNAT(false)) return false;
  setupDns.stop();
  hotspotReady = false;
  WiFi.AP.end();
  hotspotIP = IPAddress(192, 168, 4, 1);
  if (!setup) {
    const IPAddress candidates[] = {IPAddress(192,168,4,1), IPAddress(192,168,50,1), IPAddress(10,42,0,1), IPAddress(172,31,250,1)};
    bool found = false;
    for (const auto &candidate : candidates) {
      if (!setupNetworksOverlap(uint32_t(WiFi.localIP()), uint32_t(WiFi.subnetMask()), uint32_t(candidate), uint32_t(hotspotMask))) {
        hotspotIP = candidate; found = true; break;
      }
    }
    if (!found) { Serial.println("AP ERROR: no non-overlapping subnet"); hotspotReady = false; return false; }
  }
  hotspotDNS = setup ? hotspotIP : WiFi.dnsIP();
  if (!uint32_t(hotspotDNS)) hotspotDNS = IPAddress(1, 1, 1, 1);
  IPAddress lease = hotspotIP; lease[3] = 2;
  hotspotReady = WiFi.AP.begin()
    && WiFi.AP.config(hotspotIP, hotspotIP, hotspotMask, lease, hotspotDNS)
    && WiFi.AP.create(setup ? "LitterSense-Setup" : "LitterSense", setup ? "littersense" : hotspotPassword, 1, 0, 4)
    && WiFi.AP.waitStatusBits(ESP_NETIF_STARTED_BIT, 1000);
  setupActive = setup && hotspotReady;
  if (setupActive && !setupDns.start(53, "*", hotspotIP)) Serial.println("Setup DNS unavailable; open http://192.168.4.1 manually.");
  Serial.printf("AP: %s, IP=%s, ready=%d\n", setup ? "LitterSense-Setup" : "LitterSense", hotspotIP.toString().c_str(), hotspotReady);
  return hotspotReady;
}

static void connectWifi(const WifiCredentials &credentials) {
  WiFi.disconnect(false, false);
  WiFi.begin(credentials.ssid, credentials.password);
  lastReconnect = millis();
  connectionObserved = false;
}

static void startRouterWiFi() {
  Serial.printf("Arduino ESP32 package: %s\n", ESP_ARDUINO_VERSION_STR);
  WiFi.persistent(false); // Only commit tested credentials through Preferences.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false); // Retry below; never race a submitted network change.
  WiFi.setSleep(false);
  setupQueue = xQueueCreate(1, sizeof(WifiCredentials));
  snprintf(setupCsrf, sizeof(setupCsrf), "%08lx%08lx%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random());
  storageReady = wifiPrefs.begin("ls-router", false);
  if (!storageReady) Serial.println("WiFi storage ERROR: cannot open preferences");
  if (storageReady && wifiPrefs.getBytesLength("wifi") == sizeof(savedWifi)) {
    wifiPrefs.getBytes("wifi", &savedWifi, sizeof(savedWifi));
    if (!validWifiCredentials(savedWifi)) savedWifi = {};
  }
  offlineSince = millis();
  if (savedWifi.ssid[0]) connectWifi(savedWifi);
  else configureHotspot(true);
  Serial.println("WiFi: saved network will be tried for 30 seconds before setup opens.");
}

static void serviceRouterWiFi() {
  const uint32_t now = millis();
  if (setupQueue && !testingWifi && xQueueReceive(setupQueue, &pendingWifi, 0) == pdTRUE) {
    testingWifi = true;
    attemptSince = now;
    connectWifi(pendingWifi);
  }
  bool connected = WiFi.status() == WL_CONNECTED && uint32_t(WiFi.localIP()) != 0;
  if (testingWifi) connected = connected && WiFi.SSID() == pendingWifi.ssid;
  if (connected && !connectionObserved) { connectedSince = now; connectionObserved = true; }
  if (!connected) connectionObserved = false;
  const bool stable = connected && setupElapsed(now, connectedSince, WIFI_STABLE_MS);
  if (testingWifi && stable) {
    // One NVS blob keeps SSID and password together across interrupted writes.
    bool stored = wifiPrefs.putBytes("wifi", &pendingWifi, sizeof(pendingWifi)) == sizeof(pendingWifi);
    if (stored) savedWifi = pendingWifi;
    memset(&pendingWifi, 0, sizeof(pendingWifi));
    testingWifi = false;
    setupState = stored ? SetupState::Connected : SetupState::StorageError;
    connectedSince = now;
    Serial.println(stored ? "WiFi connected; credentials saved." : "WiFi ERROR: credentials could not be saved.");
  } else if (testingWifi && setupElapsed(now, attemptSince, WIFI_RETRY_MS)) {
    testingWifi = false;
    setupState = SetupState::Failed;
    memset(&pendingWifi, 0, sizeof(pendingWifi));
    WiFi.disconnect(false, false);
    if (savedWifi.ssid[0]) connectWifi(savedWifi);
    connected = false;
    offlineSince = now;
    Serial.println("WiFi setup failed; previous saved credentials retained.");
  }
  if (connected) offlineSince = now;
  if (!connected && !testingWifi && setupState == SetupState::Connected) setupState = SetupState::Idle;
  if (!connected && natEnabled) setRouterNAT(false);
  if (!testingWifi && !connected && setupElapsed(now, offlineSince, WIFI_RETRY_MS) && !setupActive) {
    setupState = SetupState::Idle;
    configureHotspot(true);
  }
  if (!testingWifi && !connected && savedWifi.ssid[0] && setupElapsed(now, lastReconnect, WIFI_RETRY_MS)) connectWifi(savedWifi);
  const bool recovered = setupState == SetupState::Idle;
  const bool accepted = setupState == SetupState::Connected;
  if (!testingWifi && stable && ((!setupActive && !hotspotReady) || (setupActive && (accepted || recovered) && setupElapsed(now, connectedSince, SETUP_SUCCESS_MS)))) {
    configureHotspot(false);
  }
  if (hotspotReady && !setupActive && connected) {
    const bool overlap = setupNetworksOverlap(uint32_t(WiFi.localIP()), uint32_t(WiFi.subnetMask()), uint32_t(hotspotIP), uint32_t(hotspotMask));
    if (natEnabled && (WiFi.localIP() != natHomeIP || overlap)) setRouterNAT(false);
    if (overlap || hotspotDNS != WiFi.dnsIP()) configureHotspot(false);
    if (hotspotReady && !natEnabled && setRouterNAT(true)) natHomeIP = WiFi.localIP();
  }
  static uint32_t lastReport = 0;
  if (setupElapsed(now, lastReport, 5000)) {
    lastReport = now;
    Serial.printf("WiFi: status=%d IP=%s | AP=%s clients=%u | setup=%d NAPT=%d\n", int(WiFi.status()), WiFi.localIP().toString().c_str(), WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum(), bool(setupActive), natEnabled);
  }
}
