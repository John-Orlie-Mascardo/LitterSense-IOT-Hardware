# Portable LitterSense Wi-Fi setup

## Current verification (September 28, 2026)

- Compiled with ESP32 Arduino core 3.3.11, ESP32S3 Dev Module, 16 MB flash, QIO, OPI PSRAM, Huge APP. Firmware after the portal socket fix: 1,034,742 bytes; static RAM: 69,752 bytes.
- Webapp validation passed: nine Settings checks, targeted ESLint, and TypeScript checking.
- Host checks pass for form decoding, preserved spaces, malformed/duplicate fields, UTF-8 SSIDs, credential sizes, open networks, timer rollover and overlapping subnets.
- On the initial build no serial ports were detected. During notification debugging COM5 appeared, but opening it was denied even outside the sandbox. **The correction is not uploaded, boot-verified, or tested on a phone/two physical networks.** Confirm that COM5 is the S3 camera and close its serial monitor before upload.
- Camera pin selection and image settings are unchanged. Controls stay on port 80 and `/stream` on port 81. The same HTTP server serves setup; there is no second listener competing for port 80.

## Phone flow

1. Upload this sketch once using the board settings above and the confirmed camera-board port. Do not flash it to the RFID or gas board.
2. Power the litterbox. With no saved Wi-Fi, `LitterSense-Setup` opens immediately. With saved Wi-Fi, it tries that network for 30 seconds, then opens setup if unavailable.
3. Join **LitterSense-Setup**, password **littersense**, on the phone. Stay connected when the phone reports no internet.
4. Open the sign-in notification, or manually visit **http://192.168.4.1/**. Phones do not always open captive portals automatically. The updated dashboard also has **Open Device Wi-Fi Setup**; it opens this same local page.
5. Enter the destination's Wi-Fi name and password. Use 2.4 GHz personal Wi-Fi, or leave the password blank for an open network. Spaces in either value are preserved. Hidden SSIDs can be typed manually. Browser-login/enterprise networks and 5 GHz-only networks are outside this flow.
6. Wait for **Connected and saved**. The firmware requires a connection and nonzero DHCP address stable for three seconds before saving one SSID/password blob in flash. This proves local Wi-Fi association, not public internet or dashboard synchronization.
7. After 15 seconds, the setup hotspot closes and the existing **LitterSense** sensor hotspot returns with its existing password. Rejoin your usual Wi-Fi on the phone. The sensor boards should reconnect automatically using DHCP.
8. Move to another network and repeat. One last successful network is remembered; there is no network-history list. No firmware edit is needed for a new SSID.

The setup form is available without internet, Firebase, or a provisioning token. A wrong password times out after 30 seconds, keeps the previous saved network and leaves setup open for correction. Passwords are neither rendered back into HTML nor logged. The setup POST requires a page-issued token and an AP-side connection; older direct cross-origin credential POSTs from the dashboard are intentionally replaced by opening the local page.

## Routing and sensor implications

- Normal operation retains AP+STA and IPv4 NAPT. Setup temporarily replaces the sensor AP; sensors cannot upload while there is no destination connection.
- The normal AP uses `192.168.4.1/24` where possible. If the destination subnet overlaps, it selects a non-overlapping /24 from `192.168.50.1`, `10.42.0.1`, or `172.31.250.1`. Sensor boards must use DHCP, not a fixed gateway or address. Setup always uses `192.168.4.1`.
- Normal DHCP advertises the destination router's DNS (fallback `1.1.1.1`). Wildcard captive DNS runs only during setup and stops when normal routing resumes.
- Network changes can briefly drop all clients because AP and station share one radio. Reopen video after reconnecting.
- Camera initialization failure does not prevent Wi-Fi setup. A camera driver crash/reset still requires its own hardware diagnosis.
- The optional dashboard account record is separate from the credentials stored on this camera. This sketch does not fetch cloud Wi-Fi passwords or automatically propagate account-record changes.
- A URL containing the PC's old LAN IP or localhost does not become portable through provisioning. For dashboard/sensor sync elsewhere, use a reachable deployed backend, or bring the PC, run the server, and configure its new reachable address. Remote camera viewing is a separate requirement. RFID and gas firmware URLs were not changed by this patch.

## Required physical acceptance checks

The missing-notification investigation found that the AP request check decoded `getsockname` as an IPv4-only address. The installed SDK enables IPv6 and HTTPD opens a dual-stack socket. The check now uses `NetworkClient::localIP(fd)`, which handles both IPv4 and IPv4-mapped IPv6; this fixes the shared gate for the page, status, provisioning and captive redirects. See [Espressif HTTPD socket setup](https://github.com/espressif/esp-idf/blob/v5.5.1/components/esp_http_server/src/httpd_main.c#L329-L355). Compilation does not confirm which firmware is running or that a particular phone shows its notification.

After uploading, forget and rejoin LitterSense-Setup to trigger a fresh phone check. From a PC connected to that setup AP, run `powershell -ExecutionPolicy Bypass -File .\test-setup.ps1` to check the form, status, wildcard DNS and Android/Apple/Windows HTTP probe redirects without changing credentials. This live check has not yet been run. The actual phone sign-in notification remains a separate acceptance check; manual access is always `http://192.168.4.1/`.

1. **First setup:** join the setup AP and load the form with phone mobile data disabled. Confirm invalid/blank SSIDs and short passwords fail without changing saved credentials.
2. **Wrong credentials:** submit a valid-length wrong password. Expect the failure message within 30 seconds and a responsive retry form. Power-cycle: the last good credentials must still work.
3. **Network A:** enter valid details, observe Connected and saved, then normal AP and `NAPT enabled` in Serial. Power-cycle and verify automatic reconnection without setup.
4. **Network B:** move away from A or turn A off. Expect setup after 30 seconds, then successful setup for B. Power-cycle and verify B reconnects.
5. **Power loss:** cut power while testing new credentials; last saved credentials must be retained. If power is lost after Connected and saved, the new credentials must survive.
6. **Loss after success:** turn off B during the 15-second success screen. Verify setup accepts another attempt rather than remaining locked on success.
7. **Address conflict:** repeat with a destination router using `192.168.4.0/24`. Verify a different normal AP subnet in Serial and fresh DHCP leases on both sensors.
8. **Internet:** on a phone connected to normal LitterSense, disable mobile data/VPN and load a fresh public HTTPS page. `NAPT enabled` alone is not proof. Optional Windows check: `powershell -ExecutionPolicy Bypass -File .\test-router.ps1`.
9. **Camera:** verify moving images from `http://DESTINATION_IP:81/stream` and `http://AP_IP:81/stream`, one viewer at a time. Verify controls on port 80. Repeat with both sensors connected.
10. **Sensors/backend:** require each board's HTTP 200 acknowledgment and matching dashboard data. A successful Wi-Fi setup does not prove this step.

## Repeatable local checks

```powershell
g++ -std=c++11 -static -Wall -Wextra -pedantic tests\wifi_setup_test.cpp -o "$env:TEMP\ls-wifi-check.exe"
& "$env:TEMP\ls-wifi-check.exe"
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,FlashMode=qio,PSRAM=opi,PartitionScheme=huge_app' --build-path "$env:TEMP\littersense-portable-wifi-build" .
```

Use the confirmed physical camera pin map and stable power supply. These compile settings match the existing S3 N16R8 checkout; they do not identify a connected board.
