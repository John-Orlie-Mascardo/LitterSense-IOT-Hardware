# Local website sync through CameraWebServer

Open `gas_sensor.ino` for the gas/ultrasonic board. Its Wi-Fi settings join
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

Uploads use the latest reading, not a historical queue. A background task handles
HTTP so website delays do not block the 100 ms ultrasonic sampling loop.
Compilation/tests do not establish a physical connection or an uploaded build.

## Ultrasonic entry confirmation

The fixed empty entrance measures 37 cm. `ultrasonic_presence.h` holds the tuning
values: 2-30 cm is detected, two consecutive readings at 100 ms intervals confirm,
and a valid reading of at least 34 cm clears the entrance indicator. No echo or
invalid readings never confirm an entry. An entrance indicator retained in the
30-34 cm band is not sufficient to confirm another entry.

Upload BOTH this sketch and the updated
`C:\Users\Admin\Downloads\littersense_rfid_test\littersense_rfid_test.ino`
to their respective dedicated boards. RFID now keeps the tag pending for up to
five seconds; it starts the website session only after ultrasonic confirmation.
Its existing remove-for-three-seconds / rescan-the-same-tag exit logic remains.

Both boards must be on the same camera hotspot with client-to-client traffic
allowed. RFID discovers `littersense-sensors` using mDNS, then POSTs a unique
32-character request ID to `/confirm-entry`. That request requires two NEW near
readings; retries do not restart its deadline. The sensor returns HTTP 202 while
waiting and HTTP 200 with the same ID after confirmation. Both requests and
responses stay on the trusted local Wi-Fi network.

The request uses `x-device-config-token`; the RFID and gas provisioning tokens
currently match. If they are changed independently, set
`LITTERSENSE_ULTRASONIC_TOKEN` in the private RFID config to the gas board token.
If mDNS is unavailable, set `LITTERSENSE_ULTRASONIC_HOST` there to the gas board's
current IP (without `http://` or a path), ideally with a DHCP reservation.
Do not use the camera IP or the PC website address for this value.

Physical checks after upload, with the website running:

1. Scan a tag with the entrance empty at about 37 cm. Expect `PENDING ENTRY`,
   then `VOID ENTRY` after five seconds, and no website entry or visit.
2. Remove the tag for three seconds before retrying. Scan and place a cat/object
   in the 2-30 cm zone within five seconds. Expect sensor `entry confirmed`,
   RFID `ENTRY`, then the website's active session after its normal heartbeat.
3. Test no echo, one isolated near reading, and a disconnected sensor board:
   none should create an entry. A held tag must not continually restart the window.
4. After a confirmed entry, remove the tag for three seconds and rescan the same
   tag. Expect `EXIT`, `SYNC: visit saved`, and one completed visit on the website.

Host check: `g++ -std=c++11 tests/presence_test.cpp -o .build/presence_test.exe`,
then run `.build/presence_test.exe`. The RFID directory has the session/timer checks.
One sensor/reader pair is supported per hotspot hostname. An entrance sensor
confirms an obstruction at the entrance, not the direction of travel.
