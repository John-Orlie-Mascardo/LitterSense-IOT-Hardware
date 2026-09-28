#pragma once
#include <cstring>

inline bool rfidTransportAllowed(const char *url, const char *ca, bool allowHttp) {
  return (strncmp(url, "https://", 8) == 0 && strlen(ca) > 0) ||
         (allowHttp && strncmp(url, "http://", 7) == 0);
}
#ifdef ARDUINO
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include "littersense_config.h"
#ifndef LITTERSENSE_ALLOW_HTTP
#define LITTERSENSE_ALLOW_HTTP false
#endif

struct RfidUpload {
  char tag[125];
  uint32_t startMs;
  uint32_t endMs;
};
static QueueHandle_t visitUploads;
static portMUX_TYPE liveMutex = portMUX_INITIALIZER_UNLOCKED;
static RfidUpload liveVisit = {};
static bool liveInside = false;
static char bootId[33];

void publishRfidState(const String &tag, uint32_t start, uint32_t now, bool active) {
  RfidUpload value = {};
  tag.toCharArray(value.tag, sizeof(value.tag));
  value.startMs = start;
  value.endMs = now;
  portENTER_CRITICAL(&liveMutex);
  liveVisit = value;
  liveInside = active;
  portEXIT_CRITICAL(&liveMutex);
  Serial.printf("SYNC TRACE: %s published, upload queue %s\n",
    active ? "entry" : "exit", visitUploads ? "available" : "unavailable");
  // ponytail: 32 visits in RAM; use a flash journal for outages spanning reboots.
  if (!active && visitUploads && xQueueSend(visitUploads, &value, 0) != pdTRUE) {
    Serial.println("SYNC ERROR: offline queue full; save the EXIT serial log.");
  }
}

static void uploadTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(LITTERSENSE_WIFI_SSID, LITTERSENSE_WIFI_PASSWORD);
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  Serial.printf("WIFI: connecting to %s, device MAC %s\n",
    LITTERSENSE_WIFI_SSID, WiFi.macAddress().c_str());
  const char *headers[] = {"x-litersense-ack"};
  uint32_t lastStart = 0;
  uint64_t startedAtMs = 0;
  int lastStatus = -1;
  bool timeReady = false, synced = false;
  for (;;) {
    Serial.printf("SYNC TRACE: worker running, queued visits %u\n",
      (unsigned)uxQueueMessagesWaiting(visitUploads));
    const int status = WiFi.status();
    if (status != lastStatus) {
      lastStatus = status;
      if (status == WL_CONNECTED) {
        Serial.printf("WIFI: connected, IP %s, posting to %s\n",
          WiFi.localIP().toString().c_str(), LITTERSENSE_SENSOR_URL);
        Serial.printf("WIFI: gateway %s, DNS %s\n",
          WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
        Serial.println("SYNC: waiting for NTP before uploads; camera uplink/NAPT must be available");
      } else {
        Serial.printf("WIFI: not connected (status %d), retrying\n", status);
      }
    }
    if (status == WL_CONNECTED && !timeReady && time(nullptr) > 1700000000) {
      timeReady = true;
      Serial.println("TIME: NTP synced");
    }
    if (status == WL_CONNECTED && timeReady) {
      RfidUpload live, completed;
      bool active;
      portENTER_CRITICAL(&liveMutex);
      live = liveVisit;
      active = liveInside;
      portEXIT_CRITICAL(&liveMutex);
      const bool pending = xQueuePeek(visitUploads, &completed, 0) == pdTRUE;
      const uint32_t now = millis();
      const uint64_t epochMs = (uint64_t)time(nullptr) * 1000;
      if (active && (!startedAtMs || lastStart != live.startMs)) {
        lastStart = live.startMs;
        startedAtMs = epochMs - (uint32_t)(now - live.startMs);
      }
      String body = "{\"deviceId\":\"" + WiFi.macAddress() + "\",\"sessionActive\":";
      body += active ? "true" : "false";
      body += ",\"activeRfidHex\":\"" + String(active ? live.tag : "") + "\"";
      body += ",\"activeSessionStartMs\":" + String((unsigned long long)startedAtMs);
      body += ",\"activeSessionDurationMs\":" + String(active ? (uint32_t)(now - live.startMs) : 0);
      body += ",\"events\":[";
      String eventId;
      if (pending) {
        eventId = String(bootId) + "_" + String(completed.startMs) + "_" + String(completed.endMs);
        body += "{\"eventId\":\"" + eventId + "\",\"status\":\"NORMAL\",\"rfidHex\":\"" + String(completed.tag);
        body += "\",\"durationMs\":" + String((uint32_t)(completed.endMs - completed.startMs));
        body += ",\"endedAtMs\":" + String((unsigned long long)(epochMs - (uint32_t)(now - completed.endMs))) + "}";
      }
      body += "]}";
      WiFiClientSecure tls;
      WiFiClient local;
      tls.setCACert(LITTERSENSE_ROOT_CA);
      HTTPClient http;
      http.setConnectTimeout(3000);
      http.setTimeout(5000);
      const bool secure = strncmp(LITTERSENSE_SENSOR_URL, "https://", 8) == 0;
      if (secure ? http.begin(tls, LITTERSENSE_SENSOR_URL)
                 : http.begin(local, LITTERSENSE_SENSOR_URL)) {
        http.collectHeaders(headers, 1);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("x-device-config-token", LITTERSENSE_CONFIG_TOKEN);
        Serial.printf("SYNC TRACE: POST starting, completed visit %s\n", pending ? "yes" : "no");
        const int code = http.POST(body);
        Serial.printf("SYNC TRACE: POST returned HTTP %d\n", code);
        if (code == 200 && !synced) {
          synced = true;
          Serial.println("SYNC: webapp reachable, heartbeat accepted");
        }
        if (code == 200 && pending && http.header(headers[0]) == eventId) {
          xQueueReceive(visitUploads, &completed, 0);
          Serial.println("SYNC: visit saved");
        } else if (code != 200 || pending) {
          Serial.printf("SYNC: pending, HTTP %d %s\n", code,
            code < 0 ? HTTPClient::errorToString(code).c_str() : "");
        }
        http.end();
      } else {
        Serial.println("SYNC: bad URL in littersense_config.h");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void beginRfidSync() {
  if (!strlen(LITTERSENSE_WIFI_SSID) || !strlen(LITTERSENSE_CONFIG_TOKEN) ||
      !rfidTransportAllowed(LITTERSENSE_SENSOR_URL, LITTERSENSE_ROOT_CA, LITTERSENSE_ALLOW_HTTP)) {
    Serial.println("SYNC disabled: configure Wi-Fi, token, and HTTPS CA or explicit local HTTP");
    return;
  }
  snprintf(bootId, sizeof(bootId), "%08lx%08lx%08lx%08lx",
    (unsigned long)esp_random(), (unsigned long)esp_random(),
    (unsigned long)esp_random(), (unsigned long)esp_random());
  visitUploads = xQueueCreate(32, sizeof(RfidUpload));
  if (!visitUploads || xTaskCreate(uploadTask, "rfid-sync", 12288, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("SYNC ERROR: unable to start upload task");
  }
}
#else
inline void beginRfidSync() {}
inline void publishRfidState(const String &, uint32_t, uint32_t, bool) {}
#endif
