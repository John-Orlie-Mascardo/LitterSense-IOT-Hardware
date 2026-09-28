#pragma once
#include <stdint.h>

constexpr uint32_t RFID_CONFIRM_TIMEOUT_MS = 5000;

#ifdef ARDUINO
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include "littersense_config.h"

// Both boards join the same hotspot. Override with the sensor board's IP if needed.
#ifndef LITTERSENSE_ULTRASONIC_HOST
#define LITTERSENSE_ULTRASONIC_HOST "littersense-sensors"
#endif
#ifndef LITTERSENSE_ULTRASONIC_TOKEN
#define LITTERSENSE_ULTRASONIC_TOKEN LITTERSENSE_CONFIG_TOKEN
#endif

struct ConfirmationRequest { uint32_t generation; uint32_t startedAt; };
static QueueHandle_t confirmationRequests;
static QueueHandle_t confirmationResults;
static uint32_t confirmationGeneration = 0;

uint32_t requestUltrasonicConfirmation(uint32_t now) {
    const ConfirmationRequest request = {++confirmationGeneration, now};
    if (confirmationRequests) xQueueOverwrite(confirmationRequests, &request);
    return request.generation;
}

bool ultrasonicConfirmed(uint32_t generation) {
    uint32_t result = 0;
    return confirmationResults && xQueueReceive(confirmationResults, &result, 0) == pdTRUE && result == generation;
}

static void confirmationTask(void *) {
    uint32_t generation = 0;
    bool delivered = false;
    bool mdnsReady = false;
    IPAddress sensorIp;
    char requestId[33] = {};
    for (;;) {
        ConfirmationRequest request;
        if (WiFi.status() != WL_CONNECTED) {
            if (mdnsReady) MDNS.end();
            mdnsReady = false;
            sensorIp = IPAddress();
        } else if (xQueuePeek(confirmationRequests, &request, 0) == pdTRUE &&
                   uint32_t(millis() - request.startedAt) < RFID_CONFIRM_TIMEOUT_MS) {
            if (request.generation != generation) {
                generation = request.generation;
                delivered = false;
                snprintf(requestId, sizeof(requestId), "%08lx%08lx%08lx%08lx",
                    (unsigned long)esp_random(), (unsigned long)esp_random(),
                    (unsigned long)esp_random(), (unsigned long)esp_random());
            }
            if (!delivered) {
                if (!sensorIp && !sensorIp.fromString(LITTERSENSE_ULTRASONIC_HOST)) {
                    if (!mdnsReady) mdnsReady = MDNS.begin("littersense-rfid");
                    if (mdnsReady) sensorIp = MDNS.queryHost(LITTERSENSE_ULTRASONIC_HOST, 200);
                }
                if (sensorIp && uint32_t(millis() - request.startedAt) < RFID_CONFIRM_TIMEOUT_MS) {
                    WiFiClient client;
                    HTTPClient http;
                    http.setConnectTimeout(300);
                    http.setTimeout(500);
                    if (http.begin(client, "http://" + sensorIp.toString() + "/confirm-entry")) {
                        http.addHeader("Content-Type", "text/plain");
                        http.addHeader("x-device-config-token", LITTERSENSE_ULTRASONIC_TOKEN);
                        const int code = http.POST(String(requestId));
                        if (code == 200 && http.getString() == requestId &&
                            uint32_t(millis() - request.startedAt) < RFID_CONFIRM_TIMEOUT_MS) {
                            xQueueOverwrite(confirmationResults, &generation);
                            delivered = true;
                        }
                        if (code < 0) sensorIp = IPAddress();
                        if (code == 403) {
                            Serial.println("ULTRASONIC: pairing token rejected; entry will be void");
                            delivered = true;
                        }
                        http.end();
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void beginUltrasonicConfirmation() {
    confirmationRequests = xQueueCreate(1, sizeof(ConfirmationRequest));
    confirmationResults = xQueueCreate(1, sizeof(uint32_t));
    if (!confirmationRequests || !confirmationResults ||
        xTaskCreate(confirmationTask, "rfid-confirm", 8192, nullptr, 1, nullptr) != pdPASS) {
        Serial.println("ULTRASONIC ERROR: confirmation unavailable; entries will be void");
    }
}
#else
// Host test transport: tests explicitly deliver a response for a request generation.
static uint32_t confirmationGeneration = 0;
static uint32_t testConfirmedGeneration = 0;
inline uint32_t requestUltrasonicConfirmation(uint32_t) { return ++confirmationGeneration; }
inline bool ultrasonicConfirmed(uint32_t generation) {
    const bool matched = testConfirmedGeneration == generation;
    testConfirmedGeneration = 0;
    return matched;
}
inline void beginUltrasonicConfirmation() {}
#endif
