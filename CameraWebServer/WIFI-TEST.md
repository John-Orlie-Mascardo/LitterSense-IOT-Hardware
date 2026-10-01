# LitterSense Wi-Fi changes and tests

## Verified locally

- Arduino IDE's configured package directory contains **esp32:esp32 3.3.11**; `arduino-cli core list` confirms it.
- After correcting the camera selection, compile/link passed with `arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,FlashMode=qio,PSRAM=opi,PartitionScheme=huge_app`: 1,005,578 bytes flash and 68,128 bytes static RAM. Upload of this correction and all hardware tests below remain unverified.
- Every installed S3 SDK variant enables `CONFIG_LWIP_IP_FORWARD=1` and `CONFIG_LWIP_IPV4_NAPT=1`. This is compiled SDK support, not sketch-level defines pretending to enable it.
- The package includes the WiFiExtender example and `WiFi.AP.enableNAPT()`. Our helper calls the underlying `esp_netif_napt_enable/disable` so Serial reports the actual error code/name even with core debug logging disabled.
- `/stream` remains on **port 81**, camera controls on port 80. `camera_pins.h`, `app_httpd.cpp`, and `camera_index.h` remain unchanged.
- After the reboot report, the matching ELF SHA256 `3f4a358fe` decoded to `ll_cam_set_pin`, called by `esp_camera_init` before Wi-Fi setup. The old `CAMERA_MODEL_ESP_EYE` selected invalid S3 PCLK GPIO25. The user identified an AIDEEPEN ESP32-S3-CAM N16R8; `board_config.h` now selects `CAMERA_MODEL_ESP32S3_EYE`, matching the published S3N16R8 mapping (https://github.com/yoursunny/esp32cam/blob/main/src/esp32cam/pins.hpp). This mapping requires hardware verification for the seller's revision. A compile-time PCLK validity assertion catches the original error; camera config is now zero-initialized.

## Arduino IDE setup

1. Reload `CameraWebServer.ino` from disk so an older IDE buffer does not overwrite these edits. For the identified N16R8 module select **ESP32S3 Dev Module**, **Flash Size: 16MB**, **Flash Mode: QIO**, **PSRAM: OPI PSRAM**, **Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)**. Leave the USB/port settings that successfully uploaded and showed Serial output. The crashing IDE build had 4MB flash and PSRAM disabled. Huge APP leaves some of the 16MB unused but needs no custom partition table for this test.
2. Keep/check the existing `ssid` and `password` in the sketch for your home's **2.4 GHz** network. They were preserved and are never printed by the added code.
3. In `wifi_router.h`, replace `CHANGE_ME_LitterSense` with your private 8-63 character hotspot password. The placeholder deliberately refuses to start the hotspot. Set the same credentials on the two sensor boards: SSID `LitterSense`, your new password, IP configuration by DHCP.
4. Use ESP32 package **3.3.11**, Verify, then upload to your confirmed S3 port. No package replacement is needed. An unsupported package fails compilation explicitly rather than silently creating an AP-only router.
5. Open Serial Monitor at **115200**, reset the board, and follow the tests below in order. No automatic upload was performed here.

## Ordered hardware tests

1. **Home connection:** expect `Home WiFi connected` and a nonzero home IP in the five-second status line. If unavailable after 20 seconds, the sketch proceeds and retries in the background. Check 2.4 GHz SSID, credentials and signal. A camera initialization failure still stops setup, as in the original sketch.
2. **Camera baseline:** from a device on the home network, open `http://HOME_IP/` and `http://HOME_IP:81/stream`. Confirm moving video, not just an HTTP response.
3. **Protected AP and DHCP:** connect a phone to `LitterSense` with the new password. Confirm an address in `192.168.4.x`, mask `255.255.255.0`, gateway `192.168.4.1`. Serial should show AP ready and an increased client count. Wrong passwords must fail. Up to four clients are allowed, leaving room for both sensors and the phone.
4. **NAPT:** expect `NAPT enabled`. Any `NAT ERROR` means routing setup has failed; AP association alone is not an internet success. Overlapping home/AP networks are rejected. If necessary change the AP IP and DHCP start address together to a separate /24 subnet.
5. **DNS:** DHCP advertises `1.1.1.1`, a real external resolver; no captive-portal or wildcard DNS is used. A Windows PC connected only to this hotspot can run `powershell -ExecutionPolicy Bypass -File .\test-router.ps1` to check DHCP DNS, DNS lookup and public HTTPS. If your home network blocks public DNS, set `hotspotDNS` to its reachable DNS server, update the check accordingly, reboot and forget/rejoin the hotspot to renew the lease. A successful DNS query on the S3 itself would not prove client DNS works.
6. **Required final internet test:** on the phone, **turn OFF mobile data**, disconnect VPN, stay on `LitterSense`, and open a fresh public website such as `https://example.com`. For diagnosis, temporarily disable custom Private DNS if enabled. Confirm a freshly loaded page. This is the internet-sharing acceptance test; firmware compilation and `NAPT enabled` are not substitutes.
7. **Stream regression:** on the hotspot open `http://192.168.4.1:81/stream`; confirm moving video. Close it, then repeat from the home network at `http://HOME_IP:81/stream`. Test one stream viewer at a time, matching the existing server design. Repeat public website loading with the camera active.
8. **Two sensors and recovery:** join both sensor boards using DHCP; expect the client count to rise to three with the phone. Turn the home router off: Serial should show disconnection and NAPT disabled while local AP access remains available. Restore the router: confirm home IP, NAPT enabled, and repeat the phone website and camera tests. A reconnect can interrupt existing TCP/video connections, which may need reopening.

The AP shares the station radio/channel. This implements IPv4 NAPT, not an IPv6 router or a promise of camera-plus-router throughput. Cloud Run and remote video changes are outside this patch.
