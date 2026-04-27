#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include "soc/rtc_cntl_reg.h"

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// ===========================
// WiFi provisioning
// ===========================
Preferences prefs;
WebServer setupServer(80);
DNSServer setupDns;

const char* setupApBaseSsid = "LitterSense-Setup";
const char* setupApPassword = "littersense";
const char* defaultDeviceConfigUrl = "";

const char* wifiPrefsNamespace = "wifi_cfg";
const char* wifiPrefsSsidKey = "ssid";
const char* wifiPrefsPassKey = "pass";
const char* wifiPrefsConfigUrlKey = "config_url";
const byte SETUP_DNS_PORT = 53;
const unsigned long CLOUD_CONFIG_SYNC_INTERVAL_MS = 300000UL;
// Default to DHCP so the device can join arbitrary owner networks.
// Set this to true only if you want a fixed IP and have matched the values below to that router.
const bool useStaticIpForTargetWifi = false;
// Keep true while testing owner setup. Set to false after the first successful hardware test.
const bool forceSetupPortalOnBoot = false;
// Keep false on ESP32-CAM unless you have a very stable supply. GPIO 4 drives the bright flash LED.
const bool useFlashLedStatusIndicator = false;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000UL;

// Optional static address for the owner Wi-Fi network.
// These values are ignored while useStaticIpForTargetWifi is false.
IPAddress localIp(192, 168, 68, 116);
IPAddress gateway(192, 168, 68, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDns(8, 8, 8, 8);
IPAddress secondaryDns(8, 8, 4, 4);
IPAddress setupApIp(192, 168, 4, 1);
IPAddress setupApSubnet(255, 255, 255, 0);

// ===========================
// Your wiring
// ===========================
#define MQ135_PIN 14
#define MQ136_PIN 13
#define RFID_RX_PIN 15
#define RFID_TX_PIN -1

// Most LM393 MQ sensor modules pull DOUT LOW when the threshold is crossed.
// If your board's DOUT goes HIGH during gas exposure, change these to HIGH.
#define MQ135_ACTIVE_LEVEL LOW
#define MQ136_ACTIVE_LEVEL LOW
#define GAS_REPORT_INTERVAL_MS 60000UL
#define RFID_PASS_DEBOUNCE_MS 5000UL
#define RFID_MAX_BYTES_PER_LOOP 32
#define RFID_DEBUG_RAW_BYTES false
#define FALSE_ENTER_MAX_MS 30000UL
#define NORMAL_SESSION_MIN_MS 120000UL
#define NORMAL_SESSION_MAX_MS 180000UL
#define NO_EXIT_TIMEOUT_MS 900000UL

// RDM6300/HW-205: TX -> GPIO 15, 9600 baud, 8N1
HardwareSerial rfid(1);
SemaphoreHandle_t serialMux;

static char rfidFrame[14];
static int rfidIndex = 0;
static unsigned long lastTagTime = 0;

volatile int latestMq135 = HIGH;
volatile int latestMq136 = HIGH;
String latestRfidHex = "";
String latestRfidCard = "";
unsigned long latestRfidTime = 0;
volatile bool rfidSessionActive = false;
String activeRfidHex = "";
String activeRfidCard = "";
unsigned long activeSessionStartTime = 0;
String latestRfidEvent = "none";
String latestSessionStatus = "NONE";
unsigned long latestSessionDurationMs = 0;
unsigned long latestSessionStartTime = 0;
unsigned long latestSessionEndTime = 0;
unsigned long completedSessionCount = 0;
unsigned long falseEntryCount = 0;
unsigned long noExitTimeoutCount = 0;
bool cameraReady = false;
bool cameraServerStarted = false;
bool setupPortalShouldStop = false;
bool setupPortalProvisioned = false;
String activeSetupApSsid = "";
bool setupPortalHasPendingCredentials = false;
String pendingSetupSsid = "";
String pendingSetupPassword = "";
String pendingSetupConfigUrl = "";
unsigned long lastCloudConfigSync = 0;
extern const int mq135ActiveLevel = MQ135_ACTIVE_LEVEL;
extern const int mq136ActiveLevel = MQ136_ACTIVE_LEVEL;
extern const unsigned long falseEnterMaxMs = FALSE_ENTER_MAX_MS;
extern const unsigned long normalSessionMinMs = NORMAL_SESSION_MIN_MS;
extern const unsigned long normalSessionMaxMs = NORMAL_SESSION_MAX_MS;
extern const unsigned long noExitTimeoutMs = NO_EXIT_TIMEOUT_MS;

void startCameraServer();
void setupLedFlash(int pin);
void sensorTask(void* pvParameters);

void setupStatusLed() {
#if defined(LED_GPIO_NUM)
  if (!useFlashLedStatusIndicator) {
    return;
  }
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
#endif
}

void setStatusLed(bool isOn) {
#if defined(LED_GPIO_NUM)
  if (!useFlashLedStatusIndicator) {
    return;
  }
  digitalWrite(LED_GPIO_NUM, isOn ? HIGH : LOW);
#endif
}

void blinkStatusLed(int count, unsigned long intervalMs) {
#if defined(LED_GPIO_NUM)
  if (!useFlashLedStatusIndicator) {
    return;
  }
  for (int i = 0; i < count; i++) {
    setStatusLed(true);
    delay(intervalMs);
    setStatusLed(false);
    delay(intervalMs);
  }
#endif
}

void updateSetupPortalIndicator() {
#if defined(LED_GPIO_NUM)
  static unsigned long lastToggleMs = 0;
  static bool isOn = false;

  unsigned long now = millis();
  if (now - lastToggleMs >= 500UL) {
    lastToggleMs = now;
    isOn = !isOn;
    setStatusLed(isOn);
  }
#endif
}

bool isProvisioningUrlConfigured(const String& configUrl) {
  String url = configUrl;
  url.trim();

  return url.length() > 0 &&
         url != "http://YOUR_PC_IP:3000/api/device-config/cfg_xxxxxxxxxxxxxxxx" &&
         !url.startsWith("http://localhost") &&
         !url.startsWith("https://localhost") &&
         !url.startsWith("http://127.0.0.1") &&
         !url.startsWith("https://127.0.0.1");
}

void printProvisioningRequirements() {
  Serial.println("Wi-Fi setup is owner-ready: no owner SSID/password is hardcoded.");
  Serial.println("If no saved Wi-Fi works, the ESP32 creates a setup network.");
  Serial.print("Setup network base name: ");
  Serial.println(setupApBaseSsid);
  Serial.print("Setup network password: ");
  Serial.println(setupApPassword);
  Serial.println("Setup page: http://192.168.4.1");
}

bool configureWifiNetwork(bool useStaticIp) {
  if (useStaticIp) {
    return WiFi.config(localIp, gateway, subnet, primaryDns, secondaryDns);
  }

  IPAddress zeroIp(0, 0, 0, 0);
  return WiFi.config(zeroIp, zeroIp, zeroIp, zeroIp, zeroIp);
}

bool connectWifi(
  const String& ssid,
  const String& password,
  bool useStaticIp,
  const char* label,
  unsigned long timeoutMs = WIFI_CONNECT_TIMEOUT_MS
) {
  if (ssid.length() == 0) {
    Serial.println("Wi-Fi SSID is empty.");
    return false;
  }

  wifi_mode_t currentMode = WiFi.getMode();
  WiFi.mode(currentMode == WIFI_AP || currentMode == WIFI_AP_STA ? WIFI_AP_STA : WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.disconnect(false, true);
  delay(200);

  if (!configureWifiNetwork(useStaticIp)) {
    Serial.println(useStaticIp
      ? "Static IP config failed; continuing with router defaults."
      : "DHCP reset failed; continuing anyway.");
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  Serial.print("Connecting to ");
  Serial.print(label);
  Serial.print(" Wi-Fi \"");
  Serial.print(ssid);
  Serial.print("\"");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println();
  Serial.print(label);
  Serial.println(" Wi-Fi failed.");
  wifi_mode_t modeAfterFailure = WiFi.getMode();
  bool keepSetupApRunning = modeAfterFailure == WIFI_AP || modeAfterFailure == WIFI_AP_STA;
  WiFi.disconnect(!keepSetupApRunning, true);
  delay(200);
  return false;
}

void saveWifiConfig(const String& ssid, const String& password) {
  prefs.begin(wifiPrefsNamespace, false);
  prefs.putString(wifiPrefsSsidKey, ssid);
  prefs.putString(wifiPrefsPassKey, password);
  prefs.end();
}

bool loadWifiConfig(String& ssid, String& password) {
  prefs.begin(wifiPrefsNamespace, true);
  ssid = prefs.getString(wifiPrefsSsidKey, "");
  password = prefs.getString(wifiPrefsPassKey, "");
  prefs.end();
  return ssid.length() > 0;
}

void saveDeviceConfigUrl(const String& configUrl) {
  String trimmedUrl = configUrl;
  trimmedUrl.trim();

  prefs.begin(wifiPrefsNamespace, false);
  if (trimmedUrl.length() > 0) {
    prefs.putString(wifiPrefsConfigUrlKey, trimmedUrl);
  } else {
    prefs.remove(wifiPrefsConfigUrlKey);
  }
  prefs.end();
}

bool loadDeviceConfigUrl(String& configUrl) {
  prefs.begin(wifiPrefsNamespace, true);
  configUrl = prefs.getString(wifiPrefsConfigUrlKey, defaultDeviceConfigUrl);
  prefs.end();
  configUrl.trim();
  return isProvisioningUrlConfigured(configUrl);
}

bool extractJsonStringField(const String& json, const char* key, String& value) {
  String pattern = "\"";
  pattern += key;
  pattern += "\"";

  int keyPos = json.indexOf(pattern);
  if (keyPos < 0) {
    return false;
  }

  int colonPos = json.indexOf(':', keyPos + pattern.length());
  if (colonPos < 0) {
    return false;
  }

  int valueStart = colonPos + 1;
  while (valueStart < json.length()) {
    char c = json[valueStart];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
      break;
    }
    valueStart++;
  }

  if (valueStart >= json.length() || json[valueStart] != '"') {
    return false;
  }

  value = "";
  bool escaping = false;

  for (int i = valueStart + 1; i < json.length(); i++) {
    char c = json[i];

    if (escaping) {
      switch (c) {
        case '"':
        case '\\':
        case '/':
          value += c;
          break;
        case 'b':
          value += '\b';
          break;
        case 'f':
          value += '\f';
          break;
        case 'n':
          value += '\n';
          break;
        case 'r':
          value += '\r';
          break;
        case 't':
          value += '\t';
          break;
        default:
          value += c;
          break;
      }

      escaping = false;
      continue;
    }

    if (c == '\\') {
      escaping = true;
      continue;
    }

    if (c == '"') {
      return true;
    }

    value += c;
  }

  return false;
}

String htmlEscape(String value) {
  value.replace("&", "&amp;");
  value.replace("\"", "&quot;");
  value.replace("<", "&lt;");
  value.replace(">", "&gt;");
  return value;
}

String makeSetupSsid() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");

  if (mac.length() >= 4) {
    return String(setupApBaseSsid) + "-" + mac.substring(mac.length() - 4);
  }

  return String(setupApBaseSsid);
}

void sendSetupCorsHeaders() {
  setupServer.sendHeader("Access-Control-Allow-Origin", "*");
  setupServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  setupServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendSetupPage(const String& message = "", bool success = false) {
  String savedUrl;
  loadDeviceConfigUrl(savedUrl);

  String html = F(
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>LitterSense Wi-Fi Setup</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:0;background:#f5f2eb;color:#1f2933;}"
    "main{max-width:440px;margin:0 auto;padding:24px 16px;}"
    ".card{background:#fff;border:1px solid #ddd7cc;border-radius:12px;padding:18px;box-shadow:0 8px 24px rgba(31,41,51,.08);}"
    "h1{font-size:24px;margin:0 0 8px;}p{font-size:14px;line-height:1.45;color:#52606d;}"
    "label{display:block;font-size:13px;font-weight:700;margin:14px 0 6px;}"
    "input{box-sizing:border-box;width:100%;border:1px solid #c8c1b5;border-radius:10px;padding:12px;font-size:16px;}"
    "button{width:100%;border:0;border-radius:10px;background:#1e6b5e;color:#fff;font-weight:700;font-size:16px;padding:13px;margin-top:18px;}"
    ".msg{border-radius:10px;padding:10px 12px;margin:12px 0;font-size:14px;}"
    ".ok{background:#e6f4ea;color:#17603a;}.err{background:#fdecea;color:#a1281f;}"
    ".hint{font-size:12px;color:#697586;word-break:break-word;}"
    "</style></head><body><main><div class='card'>"
    "<h1>LitterSense Wi-Fi Setup</h1>"
    "<p>Connect this device to the owner's Wi-Fi. Credentials are saved in the ESP32 flash, not in the Arduino sketch.</p>"
  );

  if (message.length() > 0) {
    html += "<div class='msg ";
    html += success ? "ok" : "err";
    html += "'>";
    html += htmlEscape(message);
    html += "</div>";
  }

  html += F(
    "<form method='post' action='/provision'>"
    "<label for='ssid'>Wi-Fi Name (SSID)</label>"
    "<input id='ssid' name='ssid' autocomplete='off' required>"
    "<label for='password'>Wi-Fi Password</label>"
    "<input id='password' name='password' type='password' autocomplete='current-password'>"
    "<label for='configUrl'>Device Fetch URL from the web app</label>"
    "<input id='configUrl' name='configUrl' placeholder='https://.../api/device-config/cfg_...'"
  );
  html += " value='" + htmlEscape(savedUrl) + "'>";
  html += F(
    "<p class='hint'>Optional for first connection, required for future Wi-Fi changes from Settings to sync automatically.</p>"
    "<button type='submit'>Save and Connect</button>"
    "</form>"
    "<p class='hint'>Setup network: "
  );
  html += htmlEscape(activeSetupApSsid);
  html += F(" / password: ");
  html += htmlEscape(setupApPassword);
  html += F("</p></div></main></body></html>");

  sendSetupCorsHeaders();
  setupServer.send(200, "text/html", html);
}

String readSetupArg(const char* key) {
  if (setupServer.hasArg(key)) {
    return setupServer.arg(key);
  }

  String body = setupServer.arg("plain");
  String value;
  if (body.length() > 0 && extractJsonStringField(body, key, value)) {
    return value;
  }

  return "";
}

void handleSetupStatus() {
  String response = "{";
  response += "\"setup\":true,";
  response += "\"connected\":";
  response += (WiFi.status() == WL_CONNECTED ? "true" : "false");
  response += ",\"ip\":\"";
  response += (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "");
  response += "\",\"setupSsid\":\"";
  response += activeSetupApSsid;
  response += "\"}";

  sendSetupCorsHeaders();
  setupServer.send(200, "application/json", response);
}

void handleSetupHealth() {
  String response = "LitterSense setup portal is running\n";
  response += "AP SSID: ";
  response += activeSetupApSsid;
  response += "\nAP IP: ";
  response += WiFi.softAPIP().toString();
  response += "\nClients: ";
  response += String(WiFi.softAPgetStationNum());
  response += "\n";

  sendSetupCorsHeaders();
  setupServer.send(200, "text/plain", response);
}

void redirectToSetupPage() {
  setupServer.sendHeader("Location", "http://192.168.4.1/", true);
  setupServer.send(302, "text/plain", "");
}

template <typename TClient>
bool fetchWifiConfigWithClient(
  TClient& client,
  const String& configUrl,
  String& ssid,
  String& password,
  String& deviceName
) {
  HTTPClient http;

  if (!http.begin(client, configUrl)) {
    Serial.println("Failed to open the provisioning URL.");
    return false;
  }

  http.setConnectTimeout(WIFI_CONNECT_TIMEOUT_MS);
  http.setTimeout(WIFI_CONNECT_TIMEOUT_MS);

  int code = http.GET();
  String responseBody = http.getString();
  http.end();

  if (code != HTTP_CODE_OK) {
    Serial.print("Provisioning fetch failed. HTTP ");
    Serial.println(code);
    if (responseBody.length() > 0) {
      Serial.println(responseBody);
    }
    return false;
  }

  ssid = "";
  password = "";
  deviceName = "";

  if (!extractJsonStringField(responseBody, "wifiSsid", ssid)) {
    Serial.println("Provisioning JSON missing wifiSsid.");
    Serial.println(responseBody);
    return false;
  }

  extractJsonStringField(responseBody, "wifiPassword", password);
  extractJsonStringField(responseBody, "deviceName", deviceName);

  return ssid.length() > 0;
}

bool fetchWifiConfigFromUrl(
  const String& configUrl,
  String& ssid,
  String& password,
  String& deviceName
) {
  if (!isProvisioningUrlConfigured(configUrl)) {
    Serial.println("Device config URL is missing or not reachable from the ESP32.");
    Serial.println("Paste Settings > Device Fetch URL into the setup portal.");
    return false;
  }

  Serial.print("Fetching Wi-Fi config from ");
  Serial.println(configUrl);

  if (configUrl.startsWith("https://")) {
    WiFiClientSecure client;
    client.setInsecure();
    return fetchWifiConfigWithClient(client, configUrl, ssid, password, deviceName);
  }

  WiFiClient client;
  return fetchWifiConfigWithClient(client, configUrl, ssid, password, deviceName);
}

bool fetchWifiConfig(String& ssid, String& password, String& deviceName) {
  String configUrl;
  if (!loadDeviceConfigUrl(configUrl)) {
    Serial.println("No stored Device Fetch URL. Cloud Wi-Fi sync is disabled.");
    return false;
  }

  return fetchWifiConfigFromUrl(configUrl, ssid, password, deviceName);
}

void ensureCameraServerStarted() {
  if (cameraServerStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }

  startCameraServer();
  cameraServerStarted = true;

  if (!cameraReady) {
    Serial.println("Camera routes may fail, but /sensors is available if WiFi/server started.");
  }
}

bool syncWifiConfigFromCloud(bool reconnectIfChanged) {
  String configUrl;
  if (!loadDeviceConfigUrl(configUrl)) {
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Skipping cloud Wi-Fi sync because the device is offline.");
    return false;
  }

  String fetchedSsid;
  String fetchedPassword;
  String fetchedDeviceName;
  if (!fetchWifiConfigFromUrl(configUrl, fetchedSsid, fetchedPassword, fetchedDeviceName)) {
    Serial.println("Cloud Wi-Fi sync failed; keeping the current saved config.");
    return false;
  }

  String savedSsid;
  String savedPassword;
  loadWifiConfig(savedSsid, savedPassword);

  Serial.print("Cloud config belongs to device: ");
  Serial.println(fetchedDeviceName.length() > 0 ? fetchedDeviceName : "(unnamed)");

  if (fetchedSsid == savedSsid && fetchedPassword == savedPassword) {
    Serial.println("Cloud Wi-Fi config matches saved config.");
    return true;
  }

  Serial.println("Cloud Wi-Fi config changed.");

  if (!reconnectIfChanged) {
    saveWifiConfig(fetchedSsid, fetchedPassword);
    return true;
  }

  if (connectWifi(fetchedSsid, fetchedPassword, useStaticIpForTargetWifi, "cloud")) {
    saveWifiConfig(fetchedSsid, fetchedPassword);
    Serial.println("Connected with cloud-synced Wi-Fi config.");
    return true;
  }

  Serial.println("Cloud Wi-Fi config failed. Restoring previous saved connection.");
  if (savedSsid.length() > 0) {
    connectWifi(savedSsid, savedPassword, useStaticIpForTargetWifi, "previous saved");
  }
  return false;
}

void handleSetupProvision() {
  String ssid = readSetupArg("ssid");
  String password = readSetupArg("password");
  String configUrl = readSetupArg("configUrl");

  ssid.trim();
  password.trim();
  configUrl.trim();

  if (ssid.length() == 0) {
    sendSetupPage("Enter the owner's Wi-Fi name.", false);
    return;
  }

  if (configUrl.length() > 0 && !isProvisioningUrlConfigured(configUrl)) {
    sendSetupPage("Use a real Device Fetch URL. localhost and 127.0.0.1 do not work from the ESP32.", false);
    return;
  }

  Serial.print("Setup portal received Wi-Fi SSID: ");
  Serial.println(ssid);

  pendingSetupSsid = ssid;
  pendingSetupPassword = password;
  pendingSetupConfigUrl = configUrl;
  setupPortalHasPendingCredentials = true;
  setupPortalShouldStop = true;

  String successMessage = "Received Wi-Fi details for ";
  successMessage += ssid;
  successMessage += ". The setup network will close while the ESP32 connects.";
  sendSetupPage(successMessage, true);
}

bool runSetupPortal() {
  setupPortalShouldStop = false;
  setupPortalProvisioned = false;
  setupPortalHasPendingCredentials = false;
  pendingSetupSsid = "";
  pendingSetupPassword = "";
  pendingSetupConfigUrl = "";

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  setupStatusLed();
  activeSetupApSsid = makeSetupSsid();
  WiFi.softAPConfig(setupApIp, setupApIp, setupApSubnet);

  if (!WiFi.softAP(activeSetupApSsid.c_str(), setupApPassword)) {
    Serial.println("Failed to start the LitterSense setup Wi-Fi network.");
    return false;
  }

  setupDns.start(SETUP_DNS_PORT, "*", setupApIp);

  setupServer.on("/", HTTP_GET, []() {
    sendSetupPage();
  });
  setupServer.on("/health", HTTP_GET, handleSetupHealth);
  setupServer.on("/status", HTTP_GET, handleSetupStatus);
  setupServer.on("/provision", HTTP_OPTIONS, []() {
    sendSetupCorsHeaders();
    setupServer.send(204, "text/plain", "");
  });
  setupServer.on("/provision", HTTP_POST, handleSetupProvision);
  setupServer.on("/generate_204", HTTP_GET, redirectToSetupPage);
  setupServer.on("/hotspot-detect.html", HTTP_GET, redirectToSetupPage);
  setupServer.onNotFound(redirectToSetupPage);
  setupServer.begin();

  Serial.println("Setup portal started.");
  Serial.print("Connect to Wi-Fi: ");
  Serial.println(activeSetupApSsid);
  Serial.print("Password: ");
  Serial.println(setupApPassword);
  Serial.println("Then open http://192.168.4.1");
  Serial.print("Setup AP IP: ");
  Serial.println(WiFi.softAPIP());

  unsigned long lastPortalStatusMs = 0;
  while (!setupPortalShouldStop) {
    setupDns.processNextRequest();
    setupServer.handleClient();
    updateSetupPortalIndicator();

    unsigned long now = millis();
    if (now - lastPortalStatusMs >= 5000UL) {
      lastPortalStatusMs = now;
      Serial.print("Setup portal active. Connected clients: ");
      Serial.println(WiFi.softAPgetStationNum());
    }

    delay(10);
  }

  unsigned long closeAt = millis() + 2000;
  while (millis() < closeAt) {
    setupDns.processNextRequest();
    setupServer.handleClient();
    setStatusLed(true);
    delay(10);
  }

  setupServer.stop();
  setupDns.stop();
  WiFi.softAPdisconnect(true);
  activeSetupApSsid = "";
  setStatusLed(false);

  if (!setupPortalHasPendingCredentials || pendingSetupSsid.length() == 0) {
    Serial.println("Setup portal closed without Wi-Fi credentials.");
    return false;
  }

  String ssid = pendingSetupSsid;
  String password = pendingSetupPassword;
  String configUrl = pendingSetupConfigUrl;

  pendingSetupSsid = "";
  pendingSetupPassword = "";
  pendingSetupConfigUrl = "";
  setupPortalHasPendingCredentials = false;

  Serial.print("Connecting after setup portal closed. SSID: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  delay(300);

  setStatusLed(true);
  if (!connectWifi(ssid, password, useStaticIpForTargetWifi, "setup")) {
    setStatusLed(false);
    Serial.println("Setup Wi-Fi connection failed. Reboot or wait for setup portal retry.");
    return false;
  }
  setStatusLed(false);

  saveWifiConfig(ssid, password);
  saveDeviceConfigUrl(configUrl);

  if (configUrl.length() > 0) {
    syncWifiConfigFromCloud(true);
  }

  setupPortalProvisioned = true;
  return WiFi.status() == WL_CONNECTED;
}

bool connectWithStoredOrProvisionedWifi() {
  String savedSsid;
  String savedPassword;

  if (loadWifiConfig(savedSsid, savedPassword)) {
    if (connectWifi(savedSsid, savedPassword, useStaticIpForTargetWifi, "saved")) {
      Serial.println("Connected using saved Wi-Fi config.");
      syncWifiConfigFromCloud(true);
      return true;
    }

    Serial.println("Saved Wi-Fi failed. Starting owner setup portal.");
  } else {
    Serial.println("No saved Wi-Fi config found.");
  }

  return runSetupPortal();
}

uint8_t hexPairToByte(const char* text) {
  char buffer[3] = { text[0], text[1], '\0' };
  return (uint8_t)strtoul(buffer, NULL, 16);
}

uint64_t hexTextToUint64(const char* text, int length) {
  uint64_t value = 0;

  for (int i = 0; i < length; i++) {
    char c = text[i];
    uint8_t nibble = 0;

    if (c >= '0' && c <= '9') {
      nibble = c - '0';
    } else if (c >= 'A' && c <= 'F') {
      nibble = c - 'A' + 10;
    } else if (c >= 'a' && c <= 'f') {
      nibble = c - 'a' + 10;
    } else {
      return 0;
    }

    value = (value << 4) | nibble;
  }

  return value;
}

void printUint64(uint64_t value) {
  char buffer[21];
  int index = sizeof(buffer) - 1;
  buffer[index] = '\0';

  if (value == 0) {
    Serial.print('0');
    return;
  }

  while (value > 0 && index > 0) {
    buffer[--index] = '0' + (value % 10);
    value /= 10;
  }

  Serial.print(&buffer[index]);
}

bool isHexText(const char* text, int length) {
  for (int i = 0; i < length; i++) {
    char c = text[i];
    bool valid = (c >= '0' && c <= '9') ||
                 (c >= 'A' && c <= 'F') ||
                 (c >= 'a' && c <= 'f');
    if (!valid) {
      return false;
    }
  }

  return true;
}

const char* classifySession(unsigned long durationMs) {
  if (durationMs < FALSE_ENTER_MAX_MS) {
    return "FALSE_ENTRY_IGNORED";
  }

  if (durationMs >= NORMAL_SESSION_MIN_MS && durationMs < NORMAL_SESSION_MAX_MS) {
    return "NORMAL";
  }

  if (durationMs >= NORMAL_SESSION_MAX_MS) {
    return "ABNORMAL";
  }

  return "SHORT_SESSION";
}

void printDurationSeconds(unsigned long durationMs) {
  Serial.print(durationMs / 1000);
  Serial.print("s");
}

void resetActiveRfidSession() {
  rfidSessionActive = false;
  activeRfidHex = "";
  activeRfidCard = "";
  activeSessionStartTime = 0;
}

void expireRfidSessionIfTimedOut(unsigned long now) {
  if (!rfidSessionActive || now - activeSessionStartTime < NO_EXIT_TIMEOUT_MS) {
    return;
  }

  unsigned long durationMs = now - activeSessionStartTime;

  if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
    latestRfidEvent = "NO_EXIT_TIMEOUT";
    latestSessionStatus = "NO_EXIT_TIMEOUT";
    latestSessionDurationMs = durationMs;
    latestSessionStartTime = activeSessionStartTime;
    latestSessionEndTime = now;
    falseEntryCount++;
    noExitTimeoutCount++;

    Serial.print("RFID session reset: no OUT scan after ");
    printDurationSeconds(durationMs);
    Serial.println(". Marked as NO_EXIT_TIMEOUT.");

    resetActiveRfidSession();
    xSemaphoreGive(serialMux);
  }
}

void updateRfidSessionLocked(const char* fullHex, const char* cardHex, unsigned long now) {
  latestRfidHex = fullHex;
  latestRfidCard = cardHex;
  latestRfidTime = now;

  if (!rfidSessionActive) {
    rfidSessionActive = true;
    activeRfidHex = fullHex;
    activeRfidCard = cardHex;
    activeSessionStartTime = now;
    latestRfidEvent = "ENTER";
    latestSessionStatus = "IN_PROGRESS";
    latestSessionDurationMs = 0;
    latestSessionStartTime = now;
    latestSessionEndTime = 0;

    Serial.println("RFID session: ENTER");
    Serial.print("Active card:  ");
    Serial.println(activeRfidCard);
    return;
  }

  if (activeRfidCard != cardHex) {
    latestRfidEvent = "DIFFERENT_TAG_IGNORED";
    Serial.print("RFID session ignored: active card is ");
    Serial.print(activeRfidCard);
    Serial.print(", scanned card is ");
    Serial.println(cardHex);
    return;
  }

  unsigned long durationMs = now - activeSessionStartTime;
  const char* sessionStatus = classifySession(durationMs);

  latestSessionDurationMs = durationMs;
  latestSessionStartTime = activeSessionStartTime;
  latestSessionEndTime = now;
  latestSessionStatus = sessionStatus;

  if (strcmp(sessionStatus, "FALSE_ENTRY_IGNORED") == 0) {
    latestRfidEvent = "FALSE_ENTRY_IGNORED";
    falseEntryCount++;
    Serial.print("RFID session ignored as false entry. Duration: ");
  } else {
    latestRfidEvent = "OUT";
    completedSessionCount++;
    Serial.print("RFID session: OUT. Duration: ");
  }

  printDurationSeconds(durationMs);
  Serial.print(" Status: ");
  Serial.println(sessionStatus);

  resetActiveRfidSession();
}

void printRfidTag() {
  char fullHex[11];
  char cardHex[9];
  char checksumHex[3];

  memcpy(fullHex, &rfidFrame[1], 10);
  fullHex[10] = '\0';
  memcpy(cardHex, &rfidFrame[3], 8);
  cardHex[8] = '\0';
  memcpy(checksumHex, &rfidFrame[11], 2);
  checksumHex[2] = '\0';

  if (!isHexText(fullHex, 10) || !isHexText(checksumHex, 2)) {
    Serial.println("RFID frame rejected: non-hex data");
    return;
  }

  uint8_t calculatedChecksum = 0;
  for (int i = 0; i < 10; i += 2) {
    calculatedChecksum ^= hexPairToByte(&fullHex[i]);
  }

  uint8_t receivedChecksum = hexPairToByte(checksumHex);
  uint64_t fullValue = hexTextToUint64(fullHex, 10);
  uint64_t cardValue = hexTextToUint64(cardHex, 8);

  if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
    updateRfidSessionLocked(fullHex, cardHex, millis());

    Serial.println();
    Serial.println("RFID tag detected");
    Serial.print("Full HEX:     ");
    Serial.println(fullHex);
    Serial.print("Full decimal: ");
    printUint64(fullValue);
    Serial.println();
    Serial.print("Card HEX:     ");
    Serial.println(cardHex);
    Serial.print("Card decimal: ");
    printUint64(cardValue);
    Serial.println();
    Serial.print("Checksum:     ");
    Serial.print(checksumHex);
    Serial.print(receivedChecksum == calculatedChecksum ? " OK" : " FAILED");
    Serial.print(" (calculated ");
    if (calculatedChecksum < 0x10) {
      Serial.print('0');
    }
    Serial.print(calculatedChecksum, HEX);
    Serial.println(")");
    Serial.println();
    xSemaphoreGive(serialMux);
  }
}

void handleRfidByte(uint8_t value) {
#if RFID_DEBUG_RAW_BYTES
  if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
    Serial.print("RFID RX byte: 0x");
    if (value < 0x10) {
      Serial.print('0');
    }
    Serial.println(value, HEX);
    xSemaphoreGive(serialMux);
  }
#endif

  if (value == 0x02) {
    rfidIndex = 0;
    rfidFrame[rfidIndex++] = value;
    return;
  }

  if (rfidIndex == 0) {
    return;
  }

  if (rfidIndex < (int)sizeof(rfidFrame)) {
    rfidFrame[rfidIndex++] = value;
  } else {
    rfidIndex = 0;
    return;
  }

  if (value == 0x03) {
    if (rfidIndex == 14 && (lastTagTime == 0 || millis() - lastTagTime > RFID_PASS_DEBOUNCE_MS)) {
      printRfidTag();
      lastTagTime = millis();
    } else if (rfidIndex != 14 && xSemaphoreTake(serialMux, portMAX_DELAY)) {
      Serial.print("Bad RFID frame length: ");
      Serial.println(rfidIndex);
      xSemaphoreGive(serialMux);
    }

    rfidIndex = 0;
  }
}

void printGasReading(const char* label, int pin, int activeLevel) {
  int raw = digitalRead(pin);
  bool detected = raw == activeLevel;

  Serial.print(label);
  Serial.print(" DOUT GPIO ");
  Serial.print(pin);
  Serial.print(": raw=");
  Serial.print(raw);
  Serial.print(" trigger=");
  Serial.print(activeLevel == LOW ? "LOW" : "HIGH");
  Serial.print(" status=");
  Serial.println(detected ? "GAS DETECTED" : "Clear");
}

bool startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, FRAMESIZE_QVGA);

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  return true;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(1000);
  setupStatusLed();
  blinkStatusLed(3, 120);

  serialMux = xSemaphoreCreateMutex();

  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ136_PIN, INPUT);
  pinMode(RFID_RX_PIN, INPUT_PULLUP);
  rfid.begin(9600, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);

  Serial.println();
  Serial.println("ESP32-CAM + HW-205 RDM6300 + MQ-135 + MQ-136");
  Serial.println("RFID TX must be connected to GPIO 15.");
  Serial.println("MQ-135 DOUT must be connected to GPIO 14.");
  Serial.println("MQ-136 DOUT must be connected to GPIO 13.");
  Serial.println("If raw DOUT changes during smoke but status stays Clear, flip MQ*_ACTIVE_LEVEL.");
  Serial.println("If raw DOUT never changes, adjust the MQ module potentiometer or check DOUT/AOUT wiring.");
  Serial.print("MQ-135 startup DOUT raw level: ");
  Serial.println(digitalRead(MQ135_PIN));
  Serial.print("MQ-136 startup DOUT raw level: ");
  Serial.println(digitalRead(MQ136_PIN));
  printProvisioningRequirements();

  bool wifiReady = false;
  if (forceSetupPortalOnBoot) {
    Serial.println("Force setup portal is enabled for hardware testing.");
    wifiReady = runSetupPortal();
  } else {
    wifiReady = connectWithStoredOrProvisionedWifi();
  }

  cameraReady = startCamera();
  if (!cameraReady) {
    Serial.println("Camera failed, continuing RFID/gas task only.");
  }

  if (wifiReady) {
    ensureCameraServerStarted();
  } else {
    Serial.println("Wi-Fi is unavailable. The sketch will keep retrying in loop().");
  }

  xTaskCreatePinnedToCore(
    sensorTask,
    "SensorTask",
    8192,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
      Serial.println("WiFi disconnected. Trying saved config, then provisioning...");
      xSemaphoreGive(serialMux);
    }

    if (connectWithStoredOrProvisionedWifi()) {
      ensureCameraServerStarted();
    }
  } else {
    ensureCameraServerStarted();

    unsigned long now = millis();
    if (lastCloudConfigSync == 0 || now - lastCloudConfigSync >= CLOUD_CONFIG_SYNC_INTERVAL_MS) {
      lastCloudConfigSync = now;
      syncWifiConfigFromCloud(true);
    }
  }

  delay(10000);
}

void sensorTask(void* pvParameters) {
  unsigned long lastGasCheck = millis() - GAS_REPORT_INTERVAL_MS;
  unsigned long lastRfidWaitingMessage = 0;
  unsigned long lastRfidNoiseMessage = 0;

  while (true) {
    unsigned long now = millis();
    int rfidBytesThisLoop = 0;

    while (rfid.available() && rfidBytesThisLoop < RFID_MAX_BYTES_PER_LOOP) {
      handleRfidByte((uint8_t)rfid.read());
      rfidBytesThisLoop++;
    }

    if (rfidBytesThisLoop >= RFID_MAX_BYTES_PER_LOOP && now - lastRfidNoiseMessage >= 5000) {
      lastRfidNoiseMessage = now;
      if (xSemaphoreTake(serialMux, pdMS_TO_TICKS(10))) {
        Serial.println("RFID RX is noisy; throttling reads to keep the sensor task responsive.");
        xSemaphoreGive(serialMux);
      }
    }

    expireRfidSessionIfTimedOut(now);

    if (now - lastRfidWaitingMessage >= 5000) {
      lastRfidWaitingMessage = now;
      if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
        Serial.println("Scan your cat");
        xSemaphoreGive(serialMux);
      }
    }

    if (now - lastGasCheck >= GAS_REPORT_INTERVAL_MS) {
      lastGasCheck = now;

      latestMq135 = digitalRead(MQ135_PIN);
      latestMq136 = digitalRead(MQ136_PIN);

      if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
        printGasReading("MQ-135", MQ135_PIN, MQ135_ACTIVE_LEVEL);
        printGasReading("MQ-136", MQ136_PIN, MQ136_ACTIVE_LEVEL);
        Serial.println("---");
        xSemaphoreGive(serialMux);
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
