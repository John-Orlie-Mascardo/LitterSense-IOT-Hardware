# Local website sync through CameraWebServer

Open `littersense-camera.ino` for the gas/ultrasonic board. Its Wi-Fi settings join
the camera's `LitterSense` hotspot. The private `sensor_sync_config.h` Arduino tab
contains the PC website endpoint and the existing owner provisioning token.
Do not publish that header or print its token.

The connection is sensor board -> camera hotspot/NAPT -> home LAN -> PC website.
The PC is currently `192.168.68.108`; update `SENSOR_SYNC_URL` if its IP changes.
`localhost` on the board would mean the board itself, not the PC.
This configuration uses HTTP only on the trusted local network.

1. Keep CameraWebServer powered and confirm its home Wi-Fi and NAPT status.
2. Start the website from `C:\Users\Admin\LitterSense-CSP-WebApp`:
   `npm.cmd run dev -- --hostname 0.0.0.0`
3. Reload this sketch from disk in Arduino IDE and upload it to the dedicated
   gas/ultrasonic board using its confirmed board and port settings.
4. At 115200 baud, expect `WIFI: status=3`, an IP such as `192.168.4.x`, gateway
   `192.168.4.1`, then `SYNC: gas/ultrasonic readings saved (HTTP 200)`.
5. Sign into localhost as the owner of the provisioning token. Readings should
   appear within a dashboard refresh. They become offline after 30 seconds
   without an accepted upload.

The existing `/sensors` endpoint remains available on the board. The website
prefers its separately stored pushed readings over the old polling address;
you do not need to set `ESP32_GAS_ULTRASONIC_URL` to a `192.168.4.x` address.
Camera video and RFID visit state retain their own routes/state.

If Wi-Fi status is not 3, check that the updated sketch was uploaded and the
camera hotspot is running. HTTP connection failures require checking camera
NAPT, the website listener, and Windows Firewall access on the private network.
HTTP 400/404/422 requires checking the website error and saved provisioning
settings; HTTP 503 requires checking the website's Firebase server logs.
Never post credentials with logs.

Uploads use the latest reading, not a historical queue. A bounded synchronous
HTTP request can slow sampling while the website is unavailable.
Compilation/tests do not establish a physical connection or an uploaded build.
