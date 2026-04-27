#include <WiFi.h>

const int FLASH_LED_PIN = 4;
const char* TEST_AP_SSID = "LitterSense-Setup-Test";
const char* TEST_AP_PASSWORD = "littersense";

void blink(int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(delayMs);
  }
}

void setup() {
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  blink(5, 150);

  WiFi.mode(WIFI_AP);
  bool started = WiFi.softAP(TEST_AP_SSID, TEST_AP_PASSWORD);

  Serial.println();
  Serial.println("ESP32-CAM setup-mode diagnostic");
  Serial.print("AP started: ");
  Serial.println(started ? "yes" : "no");
  Serial.print("SSID: ");
  Serial.println(TEST_AP_SSID);
  Serial.print("Password: ");
  Serial.println(TEST_AP_PASSWORD);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  blink(1, 500);
  delay(500);
}
