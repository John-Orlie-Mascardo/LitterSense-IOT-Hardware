#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <FS.h>
#include <SPIFFS.h>
#include <time.h>
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
#define ULTRASONIC_TRIG_PIN 2
#define ULTRASONIC_ECHO_PIN 4

// Most LM393 MQ sensor modules pull DOUT LOW when the threshold is crossed.
// If your board's DOUT goes HIGH during gas exposure, change these to HIGH.
#define MQ135_ACTIVE_LEVEL LOW
#define MQ136_ACTIVE_LEVEL LOW
#define GAS_REPORT_INTERVAL_MS 60000UL
#define ULTRASONIC_REPORT_INTERVAL_MS 5000UL
#define ULTRASONIC_ECHO_TIMEOUT_US 30000UL
#define ULTRASONIC_LIMIT_MM 300
#define SENSOR_SYNC_INTERVAL_MS 30000UL
#define SYNC_HTTP_TIMEOUT_MS 5000UL
#define SYNC_BATCH_MAX_EVENTS 8
#define SYNC_EVENT_LINE_MAX_LEN 512
#define SYNC_EVENTS_JSON_MAX_LEN 3200
#define SYNC_BODY_MAX_LEN 3600
#define SENSOR_SYNC_NO_PENDING 0
#define SENSOR_SYNC_SUCCESS 1
#define SENSOR_SYNC_FAILED 2
#define RFID_PASS_DEBOUNCE_MS 5000UL
#define RFID_MAX_BYTES_PER_LOOP 32
#define RFID_DEBUG_RAW_BYTES false
#define FALSE_ENTER_MAX_MS 30000UL
#define NORMAL_SESSION_MIN_MS 120000UL
#define NORMAL_SESSION_MAX_MS 180000UL
#define NO_EXIT_TIMEOUT_MS 900000UL
#define RFID_ENTRY_CONFIRMATION_WINDOW_MS 10000UL

// ESP32-CAM SD_MMC pins overlap this live sensor map, so offline replay uses onboard flash.
const bool enableFlashOfflineLogging = true;
const char* syncCursorPath = "/sync_cursor.txt";
const char* defaultSensorSyncUrl = "";

// RDM6300/HW-205: TX -> GPIO 15, 9600 baud, 8N1
HardwareSerial rfid(1);
SemaphoreHandle_t serialMux;
SemaphoreHandle_t sdMux;

static char rfidFrame[14];
static int rfidIndex = 0;
static unsigned long lastTagTime = 0;

volatile int latestMq135 = HIGH;
volatile int latestMq136 = HIGH;
volatile int latestUltrasonicDistanceMm = -1;
volatile unsigned long latestUltrasonicEchoUs = 0;
volatile unsigned long latestUltrasonicTime = 0;
volatile bool latestUltrasonicWithinLimit = false;
bool latestUltrasonicStateKnown = false;
String latestRfidHex = "";
String latestRfidCard = "";
unsigned long latestRfidTime = 0;
volatile bool rfidSessionActive = false;
String activeRfidHex = "";
String activeRfidCard = "";
unsigned long activeSessionStartTime = 0;
bool pendingRfidEntry = false;
String pendingRfidHex = "";
String pendingRfidCard = "";
String pendingRfidCardDec = "";
unsigned long pendingRfidTime = 0;
String latestRfidEvent = "none";
String latestSessionStatus = "NONE";
unsigned long latestSessionDurationMs = 0;
unsigned long latestSessionStartTime = 0;
unsigned long latestSessionEndTime = 0;
unsigned long completedSessionCount = 0;
unsigned long falseEntryCount = 0;
unsigned long noExitTimeoutCount = 0;
bool sdLoggingReady = false;
bool offlineStorageMounted = false;
bool sensorSyncUrlConfigured = false;
volatile bool sensorSyncRequested = false;
unsigned long sdLoggedEventCount = 0;
unsigned long sdSyncedEventCount = 0;
unsigned long sdWriteFailCount = 0;
unsigned long sdSyncFailCount = 0;
unsigned long lastSensorSync = 0;
char deviceId[24] = "";
char sensorSyncUrl[192] = "";
static char syncPostBody[SYNC_BODY_MAX_LEN];
static char syncEventsJson[SYNC_EVENTS_JSON_MAX_LEN];
static char syncEventLine[SYNC_EVENT_LINE_MAX_LEN];
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
extern const int ultrasonicLimitMm = ULTRASONIC_LIMIT_MM;

void startCameraServer();
void setupLedFlash(int pin);
void sensorTask(void* pvParameters);
void setupSensorPins();
void restoreSharedSensorPins();
bool initializeOfflineStorage();
void configureDeviceIdentity();
bool configureSensorSyncUrl();
void syncClockIfWifiReady();
void requestSensorSyncIfOnline();
void logRfidEvent(const char* tagHex, const char* tagDec, const char* eventName);
void logSessionEvent(const char* tagHex, unsigned long durationMs, const char* status);
void logGasEvent(int mq135Raw, int mq136Raw);
void logUltrasonicEvent(int distanceMm);
int syncLoggedEventsBatch();

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

void configureDeviceIdentity() {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  snprintf(
    deviceId,
    sizeof(deviceId),
    "ESP32-CAM-%02X%02X",
    mac[4],
    mac[5]
  );
}

bool isClockSynced() {
  time_t now = time(NULL);
  return now > 1700000000;
}

void syncClockIfWifiReady() {
  static bool timeSyncStarted = false;

  if (WiFi.status() != WL_CONNECTED || isClockSynced()) {
    return;
  }

  if (!timeSyncStarted) {
    timeSyncStarted = true;
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("NTP clock sync started for offline log timestamps.");
  }
}

unsigned long currentEventTimestamp() {
  time_t now = time(NULL);
  if (now > 1700000000) {
    return (unsigned long)now;
  }

  return millis() / 1000UL;
}

int monthNumberFromName(const char* monthName) {
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* found = strstr(months, monthName);
  if (found == NULL) {
    return 1;
  }

  return ((found - months) / 3) + 1;
}

bool compileDateYmd(char* date, size_t dateSize) {
  char monthName[4] = "";
  int day = 1;
  int year = 1970;

  if (sscanf(__DATE__, "%3s %d %d", monthName, &day, &year) != 3) {
    return false;
  }

  snprintf(date, dateSize, "%04d-%02d-%02d", year, monthNumberFromName(monthName), day);
  return true;
}

void makeDailyLogPath(char* path, size_t pathSize) {
  time_t now = time(NULL);
  struct tm timeInfo;
  char fallbackDate[11] = "1970-01-01";

  if (now > 1700000000 && localtime_r(&now, &timeInfo) != NULL) {
    snprintf(
      path,
      pathSize,
      "/log_%04d-%02d-%02d.jsonl",
      timeInfo.tm_year + 1900,
      timeInfo.tm_mon + 1,
      timeInfo.tm_mday
    );
    return;
  }

  compileDateYmd(fallbackDate, sizeof(fallbackDate));
  snprintf(path, pathSize, "/log_%s.jsonl", fallbackDate);
}

void restoreSharedSensorPins() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(RFID_RX_PIN, INPUT_PULLUP);
  rfid.begin(9600, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
}

void setupSensorPins() {
  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ136_PIN, INPUT);
  restoreSharedSensorPins();
}

bool beginSdTransaction(bool quiet) {
  if (!enableFlashOfflineLogging) {
    sdLoggingReady = false;
    return false;
  }

  if (sdMux == NULL || xSemaphoreTake(sdMux, pdMS_TO_TICKS(250)) != pdTRUE) {
    if (!quiet) {
      Serial.println("Offline storage busy; skipping this transaction.");
    }
    return false;
  }

  if (!offlineStorageMounted && !SPIFFS.begin(true)) {
    sdLoggingReady = false;
    if (!quiet) {
      Serial.println("Flash offline storage mount failed. Continuing without offline logging.");
    }
    xSemaphoreGive(sdMux);
    return false;
  }

  offlineStorageMounted = true;
  sdLoggingReady = true;
  return true;
}

void endSdTransaction() {
  if (sdMux != NULL) {
    xSemaphoreGive(sdMux);
  }
}

bool ensureDataDirectory() {
  return true;
}

bool initializeOfflineStorage() {
  if (!enableFlashOfflineLogging) {
    sdLoggingReady = false;
    Serial.println("Flash offline logging disabled.");
    return false;
  }

  if (!beginSdTransaction(false)) {
    return false;
  }

  bool ready = ensureDataDirectory();
  endSdTransaction();

  if (ready) {
    Serial.println("Flash offline logging ready.");
  } else {
    Serial.println("Flash offline storage mounted but could not be prepared.");
  }

  sdLoggingReady = ready;
  return ready;
}

bool configureSensorSyncUrl() {
  sensorSyncUrl[0] = '\0';

  if (strlen(defaultSensorSyncUrl) > 0) {
    strncpy(sensorSyncUrl, defaultSensorSyncUrl, sizeof(sensorSyncUrl) - 1);
    sensorSyncUrl[sizeof(sensorSyncUrl) - 1] = '\0';
  } else {
    String configUrl;
    if (loadDeviceConfigUrl(configUrl)) {
      int apiIndex = configUrl.indexOf("/api/");
      if (apiIndex > 0) {
        String baseUrl = configUrl.substring(0, apiIndex);
        int tokenIndex = configUrl.lastIndexOf('/');
        String configToken = tokenIndex >= 0 ? configUrl.substring(tokenIndex + 1) : "";
        int queryIndex = configToken.indexOf('?');
        if (queryIndex >= 0) {
          configToken = configToken.substring(0, queryIndex);
        }
        String sensorsUrl = baseUrl + "/api/sensors";
        if (configToken.length() > 0) {
          sensorsUrl += "?configToken=" + configToken;
        }
        strncpy(sensorSyncUrl, sensorsUrl.c_str(), sizeof(sensorSyncUrl) - 1);
        sensorSyncUrl[sizeof(sensorSyncUrl) - 1] = '\0';
      }
    }
  }

  sensorSyncUrlConfigured = sensorSyncUrl[0] != '\0';
  return sensorSyncUrlConfigured;
}

void requestSensorSyncIfOnline() {
  if (WiFi.status() == WL_CONNECTED) {
    sensorSyncRequested = true;
  }
}

bool appendJsonLineToSd(const char* jsonLine) {
  if (jsonLine == NULL || jsonLine[0] == '\0') {
    return false;
  }

  if (!enableFlashOfflineLogging) {
    return false;
  }

  if (!beginSdTransaction(true)) {
    sdWriteFailCount++;
    return false;
  }

  bool ok = ensureDataDirectory();
  char path[48];
  makeDailyLogPath(path, sizeof(path));

  if (ok) {
    File file = SPIFFS.open(path, FILE_APPEND);
    if (file) {
      ok = file.print(jsonLine) > 0 && file.print('\n') > 0;
      file.close();
    } else {
      ok = false;
    }
  }

  endSdTransaction();

  if (ok) {
    sdLoggedEventCount++;
    requestSensorSyncIfOnline();
  } else {
    sdWriteFailCount++;
  }

  return ok;
}

void uint64ToDecimalCString(uint64_t value, char* output, size_t outputSize) {
  if (outputSize == 0) {
    return;
  }

  char reversed[21];
  size_t count = 0;

  if (value == 0) {
    strncpy(output, "0", outputSize);
    output[outputSize - 1] = '\0';
    return;
  }

  while (value > 0 && count < sizeof(reversed)) {
    reversed[count++] = (char)('0' + (value % 10));
    value /= 10;
  }

  size_t outIndex = 0;
  while (count > 0 && outIndex + 1 < outputSize) {
    output[outIndex++] = reversed[--count];
  }
  output[outIndex] = '\0';
}

const char* gasStatusForRaw(int raw, int activeLevel) {
  return raw == activeLevel ? "GAS DETECTED" : "Clear";
}

void logRfidEvent(const char* tagHex, const char* tagDec, const char* eventName) {
  char line[256];
  snprintf(
    line,
    sizeof(line),
    "{\"type\":\"rfid\",\"tagHex\":\"%s\",\"tagDec\":\"%s\",\"event\":\"%s\",\"timestamp\":%lu}",
    tagHex,
    tagDec,
    eventName,
    currentEventTimestamp()
  );
  appendJsonLineToSd(line);
}

void logSessionEvent(const char* tagHex, unsigned long durationMs, const char* status) {
  char line[256];
  snprintf(
    line,
    sizeof(line),
    "{\"type\":\"session\",\"tagHex\":\"%s\",\"duration\":%lu,\"status\":\"%s\",\"timestamp\":%lu}",
    tagHex,
    durationMs,
    status,
    currentEventTimestamp()
  );
  appendJsonLineToSd(line);
}

void logGasEvent(int mq135Raw, int mq136Raw) {
  char line[320];
  snprintf(
    line,
    sizeof(line),
    "{\"type\":\"gas\",\"mq135\":{\"raw\":%d,\"status\":\"%s\"},\"mq136\":{\"raw\":%d,\"status\":\"%s\"},\"timestamp\":%lu}",
    mq135Raw,
    gasStatusForRaw(mq135Raw, MQ135_ACTIVE_LEVEL),
    mq136Raw,
    gasStatusForRaw(mq136Raw, MQ136_ACTIVE_LEVEL),
    currentEventTimestamp()
  );
  appendJsonLineToSd(line);
}

void logUltrasonicEvent(int distanceMm) {
  char line[192];
  int absDistanceMm = distanceMm < 0 ? -distanceMm : distanceMm;
  const char* sign = distanceMm < 0 ? "-" : "";

  snprintf(
    line,
    sizeof(line),
    "{\"type\":\"ultrasonic\",\"distanceCm\":%s%d.%d,\"timestamp\":%lu}",
    sign,
    absDistanceMm / 10,
    absDistanceMm % 10,
    currentEventTimestamp()
  );
  appendJsonLineToSd(line);
}

bool endsWithText(const char* text, const char* suffix) {
  size_t textLen = strlen(text);
  size_t suffixLen = strlen(suffix);

  return textLen >= suffixLen && strcmp(text + textLen - suffixLen, suffix) == 0;
}

bool findNextLogFile(char* path, size_t pathSize) {
  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) {
    if (root) {
      root.close();
    }
    return false;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      const char* name = file.name();
      if (name != NULL && endsWithText(name, ".jsonl")) {
        if (name[0] == '/') {
          strncpy(path, name, pathSize - 1);
          path[pathSize - 1] = '\0';
        } else {
          snprintf(path, pathSize, "/%s", name);
        }
        file.close();
        root.close();
        return true;
      }
    }

    file.close();
    file = root.openNextFile();
  }

  root.close();
  return false;
}

bool readSyncCursor(char* path, size_t pathSize, unsigned long* offset) {
  File file = SPIFFS.open(syncCursorPath, FILE_READ);
  if (!file) {
    return false;
  }

  char buffer[96];
  size_t len = file.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
  file.close();
  buffer[len] = '\0';

  char* separator = strchr(buffer, '|');
  if (separator == NULL) {
    return false;
  }

  *separator = '\0';
  strncpy(path, buffer, pathSize - 1);
  path[pathSize - 1] = '\0';
  *offset = strtoul(separator + 1, NULL, 10);
  return path[0] != '\0';
}

bool writeSyncCursor(const char* path, unsigned long offset) {
  File file = SPIFFS.open(syncCursorPath, FILE_WRITE);
  if (!file) {
    return false;
  }

  file.print(path);
  file.print('|');
  file.print(offset);
  file.print('\n');
  file.close();
  return true;
}

void clearSyncCursor() {
  if (SPIFFS.exists(syncCursorPath)) {
    SPIFFS.remove(syncCursorPath);
  }
}

bool postEventBatch(const char* eventsJson, int eventCount) {
  if (eventCount <= 0 || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  if (!sensorSyncUrlConfigured && !configureSensorSyncUrl()) {
    return false;
  }

  int bodyLen = snprintf(
    syncPostBody,
    sizeof(syncPostBody),
    "{\"deviceId\":\"%s\",\"events\":%s}",
    deviceId,
    eventsJson
  );

  if (bodyLen <= 0 || bodyLen >= (int)sizeof(syncPostBody)) {
    return false;
  }

  HTTPClient http;
  if (!http.begin(sensorSyncUrl)) {
    return false;
  }

  http.setConnectTimeout(SYNC_HTTP_TIMEOUT_MS);
  http.setTimeout(SYNC_HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST((uint8_t*)syncPostBody, (size_t)bodyLen);
  String responseBody = http.getString();
  http.end();

  if (code == HTTP_CODE_OK) {
    if (xSemaphoreTake(serialMux, pdMS_TO_TICKS(100))) {
      Serial.print("Synced ");
      Serial.print(eventCount);
      Serial.println(" SD event(s) to web app.");
      xSemaphoreGive(serialMux);
    }
    return true;
  }

  sdSyncFailCount++;
  if (xSemaphoreTake(serialMux, pdMS_TO_TICKS(100))) {
    Serial.print("Sensor sync failed. HTTP ");
    Serial.println(code);
    if (responseBody.length() > 0) {
      Serial.println(responseBody);
    }
    xSemaphoreGive(serialMux);
  }
  return false;
}

int syncLoggedEventsBatch() {
  if (!enableFlashOfflineLogging) {
    return SENSOR_SYNC_NO_PENDING;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return SENSOR_SYNC_FAILED;
  }

  if (!sensorSyncUrlConfigured && !configureSensorSyncUrl()) {
    return SENSOR_SYNC_FAILED;
  }

  char logPath[64] = "";
  unsigned long startOffset = 0;
  unsigned long endOffset = 0;
  unsigned long fileSize = 0;
  size_t eventsLen = 1;
  int eventCount = 0;

  syncEventsJson[0] = '[';
  syncEventsJson[1] = '\0';

  if (!beginSdTransaction(true)) {
    return SENSOR_SYNC_FAILED;
  }

  bool hasFile = readSyncCursor(logPath, sizeof(logPath), &startOffset);
  if (!hasFile || !SPIFFS.exists(logPath)) {
    startOffset = 0;
    hasFile = findNextLogFile(logPath, sizeof(logPath));
  }

  if (!hasFile) {
    clearSyncCursor();
    endSdTransaction();
    return SENSOR_SYNC_NO_PENDING;
  }

  File file = SPIFFS.open(logPath, FILE_READ);
  if (!file) {
    clearSyncCursor();
    endSdTransaction();
    return SENSOR_SYNC_FAILED;
  }

  fileSize = file.size();
  if (startOffset > fileSize) {
    startOffset = 0;
  }
  file.seek(startOffset);
  endOffset = startOffset;

  while (file.available() && eventCount < SYNC_BATCH_MAX_EVENTS) {
    size_t len = file.readBytesUntil('\n', syncEventLine, sizeof(syncEventLine) - 1);
    syncEventLine[len] = '\0';

    while (len > 0 && (syncEventLine[len - 1] == '\r' || syncEventLine[len - 1] == '\n')) {
      syncEventLine[--len] = '\0';
    }

    if (len == 0) {
      endOffset = file.position();
      continue;
    }

    size_t needed = len + (eventCount > 0 ? 1 : 0);
    if (eventsLen + needed + 2 >= sizeof(syncEventsJson)) {
      break;
    }

    if (eventCount > 0) {
      syncEventsJson[eventsLen++] = ',';
    }

    memcpy(syncEventsJson + eventsLen, syncEventLine, len);
    eventsLen += len;
    syncEventsJson[eventsLen] = '\0';
    eventCount++;
    endOffset = file.position();
  }

  file.close();
  syncEventsJson[eventsLen++] = ']';
  syncEventsJson[eventsLen] = '\0';
  endSdTransaction();

  if (eventCount == 0) {
    if (beginSdTransaction(true)) {
      File stale = SPIFFS.open(logPath, FILE_READ);
      unsigned long latestSize = stale ? stale.size() : 0;
      if (stale) {
        stale.close();
      }
      if (latestSize <= startOffset) {
        SPIFFS.remove(logPath);
        clearSyncCursor();
      }
      endSdTransaction();
    }
    return SENSOR_SYNC_NO_PENDING;
  }

  if (!postEventBatch(syncEventsJson, eventCount)) {
    return SENSOR_SYNC_FAILED;
  }

  if (!beginSdTransaction(true)) {
    return SENSOR_SYNC_FAILED;
  }

  File updatedFile = SPIFFS.open(logPath, FILE_READ);
  unsigned long updatedSize = updatedFile ? updatedFile.size() : 0;
  if (updatedFile) {
    updatedFile.close();
  }

  if (updatedSize <= endOffset) {
    SPIFFS.remove(logPath);
    clearSyncCursor();
  } else {
    writeSyncCursor(logPath, endOffset);
  }

  endSdTransaction();
  sdSyncedEventCount += eventCount;
  return SENSOR_SYNC_SUCCESS;
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
  WiFi.setAutoReconnect(true);
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

bool reconnectSavedWifiOnly() {
  String savedSsid;
  String savedPassword;

  if (!loadWifiConfig(savedSsid, savedPassword)) {
    Serial.println("No saved Wi-Fi config found. Setup portal is only opened from the boot setup path.");
    return false;
  }

  if (connectWifi(savedSsid, savedPassword, useStaticIpForTargetWifi, "saved reconnect")) {
    Serial.println("Reconnected using saved Wi-Fi config.");
    syncWifiConfigFromCloud(true);
    return true;
  }

  Serial.println("Saved Wi-Fi reconnect failed. Will retry without opening setup portal.");
  return false;
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

void resetPendingRfidEntry() {
  pendingRfidEntry = false;
  pendingRfidHex = "";
  pendingRfidCard = "";
  pendingRfidCardDec = "";
  pendingRfidTime = 0;
}

void startPendingRfidEntryLocked(
  const char* fullHex,
  const char* cardHex,
  const char* cardDec,
  unsigned long now
) {
  pendingRfidEntry = true;
  pendingRfidHex = fullHex;
  pendingRfidCard = cardHex;
  pendingRfidCardDec = cardDec;
  pendingRfidTime = now;
  latestRfidEvent = "PENDING_ENTER";
  latestSessionStatus = "NONE";
  latestSessionDurationMs = 0;
  latestSessionStartTime = 0;
  latestSessionEndTime = 0;

  Serial.println("RFID pending: waiting for ultrasonic confirmation within 10s.");
  Serial.print("Pending card: ");
  Serial.println(pendingRfidCard);
}

bool expirePendingRfidEntryIfTimedOut(unsigned long now) {
  if (!pendingRfidEntry || now - pendingRfidTime <= RFID_ENTRY_CONFIRMATION_WINDOW_MS) {
    return false;
  }

  if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
    latestRfidEvent = "UNCONFIRMED_ENTRY_IGNORED";
    latestSessionStatus = "NONE";
    falseEntryCount++;

    Serial.print("RFID entry rejected: ultrasonic did not confirm within ");
    Serial.print(RFID_ENTRY_CONFIRMATION_WINDOW_MS / 1000UL);
    Serial.println("s.");

    resetPendingRfidEntry();
    xSemaphoreGive(serialMux);
  }

  return true;
}

bool confirmPendingRfidEntryWithUltrasonicLocked(
  unsigned long now,
  char* confirmedCardHex,
  size_t confirmedCardHexSize,
  char* confirmedCardDec,
  size_t confirmedCardDecSize
) {
  if (!pendingRfidEntry) {
    return false;
  }

  if (now - pendingRfidTime > RFID_ENTRY_CONFIRMATION_WINDOW_MS) {
    latestRfidEvent = "UNCONFIRMED_ENTRY_IGNORED";
    latestSessionStatus = "NONE";
    falseEntryCount++;
    resetPendingRfidEntry();
    return false;
  }

  if (rfidSessionActive) {
    resetPendingRfidEntry();
    return false;
  }

  strncpy(confirmedCardHex, pendingRfidCard.c_str(), confirmedCardHexSize - 1);
  confirmedCardHex[confirmedCardHexSize - 1] = '\0';
  strncpy(confirmedCardDec, pendingRfidCardDec.c_str(), confirmedCardDecSize - 1);
  confirmedCardDec[confirmedCardDecSize - 1] = '\0';

  rfidSessionActive = true;
  activeRfidHex = pendingRfidHex;
  activeRfidCard = pendingRfidCard;
  activeSessionStartTime = now;
  latestRfidEvent = "ENTER";
  latestSessionStatus = "IN_PROGRESS";
  latestSessionDurationMs = 0;
  latestSessionStartTime = now;
  latestSessionEndTime = 0;

  Serial.println("RFID session: ENTER confirmed by ultrasonic.");
  Serial.print("Active card:  ");
  Serial.println(activeRfidCard);

  resetPendingRfidEntry();
  return true;
}

void expireRfidSessionIfTimedOut(unsigned long now) {
  if (!rfidSessionActive || now - activeSessionStartTime < NO_EXIT_TIMEOUT_MS) {
    return;
  }

  unsigned long durationMs = now - activeSessionStartTime;
  char timedOutCard[9] = "";
  bool shouldLogTimeout = false;

  if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
    strncpy(timedOutCard, activeRfidCard.c_str(), sizeof(timedOutCard) - 1);
    timedOutCard[sizeof(timedOutCard) - 1] = '\0';

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
    shouldLogTimeout = timedOutCard[0] != '\0';
    xSemaphoreGive(serialMux);
  }

  if (shouldLogTimeout) {
    logSessionEvent(timedOutCard, durationMs, "NO_EXIT_TIMEOUT");
  }
}

void updateRfidSessionLocked(const char* fullHex, const char* cardHex, const char* cardDec, unsigned long now) {
  latestRfidHex = fullHex;
  latestRfidCard = cardHex;
  latestRfidTime = now;

  if (!rfidSessionActive) {
    startPendingRfidEntryLocked(fullHex, cardHex, cardDec, now);
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
  char cardDecText[21];
  char rfidEventName[32] = "none";
  char sessionStatusName[32] = "NONE";
  unsigned long sessionDurationMs = 0;
  bool shouldLogRfid = false;
  bool shouldLogSession = false;

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
  uint64ToDecimalCString(cardValue, cardDecText, sizeof(cardDecText));

  if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
    updateRfidSessionLocked(fullHex, cardHex, cardDecText, millis());
    strncpy(rfidEventName, latestRfidEvent.c_str(), sizeof(rfidEventName) - 1);
    rfidEventName[sizeof(rfidEventName) - 1] = '\0';
    strncpy(sessionStatusName, latestSessionStatus.c_str(), sizeof(sessionStatusName) - 1);
    sessionStatusName[sizeof(sessionStatusName) - 1] = '\0';
    sessionDurationMs = latestSessionDurationMs;
    shouldLogRfid = strcmp(rfidEventName, "OUT") == 0 ||
                    strcmp(rfidEventName, "FALSE_ENTRY_IGNORED") == 0;
    shouldLogSession = strcmp(rfidEventName, "OUT") == 0 ||
                       strcmp(rfidEventName, "FALSE_ENTRY_IGNORED") == 0;

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

  if (shouldLogRfid) {
    logRfidEvent(cardHex, cardDecText, rfidEventName);
  }

  if (shouldLogSession) {
    logSessionEvent(cardHex, sessionDurationMs, sessionStatusName);
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

int readUltrasonicDistanceMm(unsigned long& echoDurationUs) {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  echoDurationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_ECHO_TIMEOUT_US);
  if (echoDurationUs == 0) {
    return -1;
  }

  return (int)((echoDurationUs * 343UL) / 2000UL);
}

void printUltrasonicReading(int distanceMm, unsigned long echoDurationUs) {
  Serial.print("HC-SR04 Trig GPIO ");
  Serial.print(ULTRASONIC_TRIG_PIN);
  Serial.print(" Echo GPIO ");
  Serial.print(ULTRASONIC_ECHO_PIN);
  Serial.print(": ");

  if (distanceMm < 0) {
    Serial.println("NO ECHO");
    return;
  }

  if (distanceMm <= ULTRASONIC_LIMIT_MM) {
    Serial.print(distanceMm / 10);
    Serial.print(".");
    Serial.print(distanceMm % 10);
    Serial.print(" cm");
  } else {
    Serial.print(">");
    Serial.print(ULTRASONIC_LIMIT_MM / 10);
    Serial.print(".");
    Serial.print(ULTRASONIC_LIMIT_MM % 10);
    Serial.print(" cm");
  }

  Serial.print(" echoUs=");
  Serial.print(echoDurationUs);
  Serial.print(" limit=");
  Serial.println(distanceMm <= ULTRASONIC_LIMIT_MM ? "WITHIN_30CM" : "CLEAR");
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
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
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
  sdMux = xSemaphoreCreateMutex();
  configureDeviceIdentity();

  Serial.println();
  Serial.println("ESP32-CAM + HW-205 RDM6300 + MQ-135 + MQ-136 + HC-SR04");
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.println("RFID TX must be connected to GPIO 15.");
  Serial.println("MQ-135 DOUT must be connected to GPIO 14.");
  Serial.println("MQ-136 DOUT must be connected to GPIO 13.");
  Serial.println("HC-SR04 Trig must be connected to GPIO 2.");
  Serial.println("HC-SR04 Echo must be connected to GPIO 4 through a voltage divider.");
  Serial.println("HC-SR04 limit is 30cm or closer.");
  Serial.println("GPIO 4 is used for HC-SR04 Echo, so camera flash LED control is disabled.");
  Serial.println("If raw DOUT changes during smoke but status stays Clear, flip MQ*_ACTIVE_LEVEL.");
  Serial.println("If raw DOUT never changes, adjust the MQ module potentiometer or check DOUT/AOUT wiring.");

  cameraReady = startCamera();
  if (!cameraReady) {
    Serial.println("Camera failed, continuing RFID/gas task only.");
  }

  // GPIO 4 is shared with the ESP32-CAM flash LED, so claim it for HC-SR04 only after camera init.
  setupSensorPins();

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

  initializeOfflineStorage();
  configureSensorSyncUrl();
  if (sensorSyncUrlConfigured) {
    Serial.print("Sensor sync endpoint: ");
    Serial.println(sensorSyncUrl);
  } else {
    Serial.println("Sensor sync endpoint is not configured. Set a Device Fetch URL from the web app.");
  }
  syncClockIfWifiReady();

  if (wifiReady) {
    ensureCameraServerStarted();
  } else {
    Serial.println("Wi-Fi is unavailable. The sketch will keep retrying in loop().");
  }

  xTaskCreatePinnedToCore(
    sensorTask,
    "SensorTask",
    16384,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
  unsigned long now = millis();
  static unsigned long lastWifiRetry = 0;

  if (WiFi.status() != WL_CONNECTED) {
    if (lastWifiRetry == 0 || now - lastWifiRetry >= 10000UL) {
      lastWifiRetry = now;
      if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
        Serial.println("WiFi disconnected. Retrying saved config without opening setup portal...");
        xSemaphoreGive(serialMux);
      }

      if (reconnectSavedWifiOnly()) {
        configureSensorSyncUrl();
        syncClockIfWifiReady();
        sensorSyncRequested = true;
        ensureCameraServerStarted();
      }
    }
  } else {
    lastWifiRetry = 0;
    ensureCameraServerStarted();
    syncClockIfWifiReady();

    if (lastCloudConfigSync == 0 || now - lastCloudConfigSync >= CLOUD_CONFIG_SYNC_INTERVAL_MS) {
      lastCloudConfigSync = now;
      if (syncWifiConfigFromCloud(true)) {
        configureSensorSyncUrl();
      }
    }

    if (sensorSyncRequested || lastSensorSync == 0 || now - lastSensorSync >= SENSOR_SYNC_INTERVAL_MS) {
      lastSensorSync = now;
      int syncResult = syncLoggedEventsBatch();
      if (syncResult == SENSOR_SYNC_SUCCESS) {
        sensorSyncRequested = true;
      } else if (syncResult == SENSOR_SYNC_NO_PENDING) {
        sensorSyncRequested = false;
      }
    }
  }

  delay(1000);
}

void sensorTask(void* pvParameters) {
  unsigned long lastGasCheck = millis() - GAS_REPORT_INTERVAL_MS;
  unsigned long lastUltrasonicCheck = millis() - ULTRASONIC_REPORT_INTERVAL_MS;
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

    expirePendingRfidEntryIfTimedOut(now);
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

      logGasEvent(latestMq135, latestMq136);
    }

    if (now - lastUltrasonicCheck >= ULTRASONIC_REPORT_INTERVAL_MS) {
      lastUltrasonicCheck = now;

      unsigned long echoDurationUs = 0;
      int distanceMm = readUltrasonicDistanceMm(echoDurationUs);
      unsigned long ultrasonicNow = millis();
      bool ultrasonicStateChanged = false;
      bool shouldLogConfirmedRfid = false;
      char confirmedCardHex[9] = "";
      char confirmedCardDec[21] = "";

      if (xSemaphoreTake(serialMux, portMAX_DELAY)) {
        bool wasStateKnown = latestUltrasonicStateKnown;
        bool previousWithinLimit = latestUltrasonicWithinLimit;
        bool currentWithinLimit = distanceMm >= 0 && distanceMm <= ULTRASONIC_LIMIT_MM;

        latestUltrasonicDistanceMm = distanceMm;
        latestUltrasonicEchoUs = echoDurationUs;
        latestUltrasonicTime = ultrasonicNow;
        latestUltrasonicWithinLimit = currentWithinLimit;
        latestUltrasonicStateKnown = true;
        ultrasonicStateChanged = wasStateKnown
          ? previousWithinLimit != currentWithinLimit
          : currentWithinLimit;

        if (currentWithinLimit) {
          shouldLogConfirmedRfid = confirmPendingRfidEntryWithUltrasonicLocked(
            ultrasonicNow,
            confirmedCardHex,
            sizeof(confirmedCardHex),
            confirmedCardDec,
            sizeof(confirmedCardDec)
          );
        }

        printUltrasonicReading(distanceMm, echoDurationUs);
        xSemaphoreGive(serialMux);
      }

      if (ultrasonicStateChanged) {
        logUltrasonicEvent(distanceMm);
      }

      if (shouldLogConfirmedRfid) {
        logRfidEvent(confirmedCardHex, confirmedCardDec, "ENTER");
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
