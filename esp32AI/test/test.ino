#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

// --- REPLACE THESE WITH YOUR DETAILS ---
const char* ssid = "AUSTRIAWIFI";
const char* password = "Joshuaaustria_19";
// Put your laptop's IP address here, keeping the :5000/upload at the end
String serverName = "http://192.168.68.108:5000/upload"; 

// Camera pin definitions for the AI Thinker module
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setup() {
  Serial.begin(115200);

  // 1. Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // 2. Configure the Camera
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
  config.frame_size = FRAMESIZE_VGA; // 640x480 resolution
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  // For testing, we will take a photo every 10 seconds.
  // In your final build, this will only trigger when the main ESP32 detects motion.
  takeAndSendPhoto();
  delay(10000); 
}

void takeAndSendPhoto() {
  Serial.println("Taking photo...");
  camera_fb_t * fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Send the image to the laptop
  HTTPClient http;
  http.begin(serverName);
  
  // Tell the server we are sending raw JPEG data, not a web form
  http.addHeader("Content-Type", "image/jpeg");

  // Send the raw image buffer directly
  int httpResponseCode = http.sendRequest("POST", fb->buf, fb->len);

  if (httpResponseCode > 0) {
    Serial.printf("Server Response: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error code: %d\n", httpResponseCode);
  }

  // Free the camera memory and close the connection
  http.end();
  esp_camera_fb_return(fb);
}