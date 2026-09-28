#pragma once

// Keep this file local; do not commit it.
#define LITTERSENSE_WIFI_SSID "LitterSense"
#define LITTERSENSE_WIFI_PASSWORD "1234567889"

// Join CameraWebServer hotspot using DHCP (gateway 192.168.4.1).
// The URL below is the WEBSITE PC on the home LAN, reached through camera NAPT.
// Do not replace it with the camera or RFID board IP:
#define LITTERSENSE_ALLOW_HTTP true // Trusted local Wi-Fi only; use HTTPS outside this LAN.
#define LITTERSENSE_SENSOR_URL "http://192.168.68.108:3000/api/sensors"

// Copy the saved Provisioning Token from your web app:
#define LITTERSENSE_CONFIG_TOKEN "cfg_8aebbed921ef4e7a8e6befe9d5a3e919"

// Unused for HTTP. HTTPS requires the endpoint's trusted root CA.
#define LITTERSENSE_ROOT_CA ""
