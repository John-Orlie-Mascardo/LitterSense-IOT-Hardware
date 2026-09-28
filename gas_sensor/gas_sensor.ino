#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include "sensor_sync_config.h"
#include "ultrasonic_presence.h"

const char* WIFI_SSID = "LitterSense";
const char* WIFI_PASSWORD = "1234567889";
WebServer server(80);
String sensorJson = "{}";
unsigned long lastSampleMs = 0;
bool wasConnected = false;
UltrasonicPresence presence;
String confirmationId;
struct SensorUpload { char json[512]; };
QueueHandle_t sensorUploads;

// MQ sensor pins
constexpr int MQ135_PIN = 13;
constexpr int MQ136_PIN = 14;

// HC-SR04 pins
constexpr int TRIG_PIN = 15;
constexpr int ECHO_PIN = 2;

void syncSensorReadings() {
    static uint32_t lastAttempt = 0;
    static uint32_t lastReconnect = 0;
    const uint32_t now = millis();
    if (now - lastAttempt < 5000) return;
    lastAttempt = now;
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("SYNC: waiting for LitterSense Wi-Fi");
        if (now - lastReconnect >= 30000) {
            lastReconnect = now;
            WiFi.reconnect();
        }
        return;
    }
    if (!strlen(SENSOR_CONFIG_TOKEN) || strncmp(SENSOR_SYNC_URL, "http://", 7) != 0) {
        Serial.println("SYNC: configure the local HTTP website URL and provisioning token in sensor_sync_config.h");
        return;
    }
    SensorUpload upload;
    if (!sensorUploads || xQueuePeek(sensorUploads, &upload, 0) != pdTRUE) return;
    // Uploads run separately so a slow website cannot pause entrance detection.
    WiFiClient client;
    HTTPClient http;
    http.setConnectTimeout(2000);
    http.setTimeout(3000);
    if (!http.begin(client, SENSOR_SYNC_URL)) {
        Serial.println("SYNC: invalid website URL");
        return;
    }
    const char* headers[] = {"x-litersense-ack"};
    http.collectHeaders(headers, 1);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-device-config-token", SENSOR_CONFIG_TOKEN);
    const int code = http.POST(String(upload.json));
    if (code == 200 && http.header(headers[0]) == "gas-ultrasonic") {
        Serial.println("SYNC: gas/ultrasonic readings saved (HTTP 200)");
    } else {
        Serial.printf("SYNC: upload failed, HTTP %d %s\n", code,
            code < 0 ? HTTPClient::errorToString(code).c_str() : "(check website receiver and provisioning token)");
    }
    http.end();
}

void sensorUploadTask(void *) {
    for (;;) {
        syncSensorReadings();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void handleConfirmation() {
    server.sendHeader("Cache-Control", "no-store");
    if (!strlen(SENSOR_CONFIG_TOKEN) || server.header("x-device-config-token") != SENSOR_CONFIG_TOKEN) {
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    const String id = server.arg("plain");
    if (id.length() != 32) {
        server.send(400, "text/plain", "Expected 32 hexadecimal characters");
        return;
    }
    for (size_t i = 0; i < id.length(); ++i) {
        if (!isxdigit(static_cast<unsigned char>(id[i]))) {
            server.send(400, "text/plain", "Invalid request ID");
            return;
        }
    }
    if (id != confirmationId) {
        confirmationId = id;
        presence.request(millis());
        Serial.println("ULTRASONIC: RFID confirmation requested");
    }
    // Retries keep the same deadline and cannot reuse another entry's confirmation.
    server.send(presence.accepted(millis()) ? 200 : 202, "text/plain", id);
}

void setup() {
    Serial.begin(115200);

    // MQ digital outputs
    pinMode(MQ135_PIN, INPUT);
    pinMode(MQ136_PIN, INPUT);

    // Ultrasonic sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    delay(2000);
    Serial.println("\n=== THREE-SENSOR TEST STARTED ===");
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    Serial.printf("WIFI: connecting to %s\n", WIFI_SSID);
    Serial.printf("SYNC: website endpoint %s\n", SENSOR_SYNC_URL);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    server.on("/sensors", HTTP_GET, []() {
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", sensorJson);
    });
    const char* requestHeaders[] = {"x-device-config-token"};
    server.collectHeaders(requestHeaders, 1);
    server.on("/confirm-entry", HTTP_POST, handleConfirmation);
    server.begin();
    sensorUploads = xQueueCreate(1, sizeof(SensorUpload));
    if (!sensorUploads || xTaskCreate(sensorUploadTask, "sensor-sync", 8192, nullptr, 1, nullptr) != pdPASS) {
        Serial.println("SYNC ERROR: unable to start sensor upload task");
    }
}

void loop() {
    server.handleClient();
    bool connected = WiFi.status() == WL_CONNECTED;
    if (connected && !wasConnected) {
        if (!MDNS.begin("littersense-sensors")) Serial.println("ULTRASONIC: mDNS startup failed");
        Serial.print("Sensor URL: http://");
        Serial.print(WiFi.localIP());
        Serial.println("/sensors");
        Serial.printf("WIFI: gateway %s, signal %d dBm\n",
            WiFi.gatewayIP().toString().c_str(), WiFi.RSSI());
        Serial.print("Sensor board MAC: ");
        Serial.println(WiFi.macAddress());
    }
    if (!connected && wasConnected) MDNS.end();
    wasConnected = connected;
    if (lastSampleMs != 0 && millis() - lastSampleMs < PRESENCE_SAMPLE_MS) {
        delay(1);
        return;
    }
    lastSampleMs = millis();
    // MQ modules are active LOW:
    // LOW = LED on / threshold detected
    // HIGH = LED off / clear
    bool mq135Detected = digitalRead(MQ135_PIN) == LOW;
    bool mq136Detected = digitalRead(MQ136_PIN) == LOW;

    // Send a 10-microsecond trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    // Wait for the Echo pulse, with a 30 ms timeout
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    const float distanceCm = duration == 0 ? -1.0f : duration * 0.0343f / 2.0f;
    const bool previouslyConfirmed = presence.confirmed;
    presence.sample(distanceCm, millis());
    if (!previouslyConfirmed && presence.confirmed) Serial.println("ULTRASONIC: entry confirmed (two readings in 2-30 cm)");

    sensorJson = "{\"source\":\"gas-ultrasonic\",\"mq135\":\"";
    sensorJson += mq135Detected ? "Gas Detected" : "Clear";
    sensorJson += "\",\"mq136\":\"";
    sensorJson += mq136Detected ? "Gas Detected" : "Clear";
    sensorJson += "\",\"mq135Raw\":";
    sensorJson += mq135Detected ? "0" : "1";
    sensorJson += ",\"mq136Raw\":";
    sensorJson += mq136Detected ? "0" : "1";
    sensorJson += ",\"distanceCm\":";
    sensorJson += duration == 0 ? String("null") : String(duration * 0.0343f / 2.0f, 1);
    sensorJson += ",\"entranceOccupied\":";
    sensorJson += presence.occupied ? "true" : "false";
    sensorJson += "}";
    SensorUpload upload = {};
    sensorJson.toCharArray(upload.json, sizeof(upload.json));
    if (sensorUploads) xQueueOverwrite(sensorUploads, &upload);

    static uint32_t lastLogMs = 0;
    if (uint32_t(millis() - lastLogMs) < 1000) return;
    lastLogMs = millis();

    Serial.println("\n------------------------------");
    Serial.printf("WIFI: status=%d (3=connected), IP=%s\n",
        int(WiFi.status()), WiFi.localIP().toString().c_str());

    Serial.print("MQ-135: ");
    Serial.println(
        mq135Detected
            ? "DETECTED (LED ON)"
            : "CLEAR (LED OFF)"
    );

    Serial.print("MQ-136: ");
    Serial.println(
        mq136Detected
            ? "DETECTED (LED ON)"
            : "CLEAR (LED OFF)"
    );

    Serial.print("HC-SR04: ");

    if (duration == 0) {
        Serial.println("NO ECHO");
    } else {
        float distanceCm = duration * 0.0343f / 2.0f;

        Serial.print(distanceCm, 1);
        Serial.println(" cm");
    }

}
