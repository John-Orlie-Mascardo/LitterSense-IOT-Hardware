# LitterSense RFID integration

A scan now creates a pending entry. The gas/ultrasonic board must confirm two
fresh readings in the 2-30 cm zone within five seconds before a session opens.
Otherwise the entry is void, with no active session or completed visit uploaded.
Three seconds of confirmed no-tag replies rearm the reader after confirmation or
rejection. The next scan of the same tag closes a confirmed session as before.
Elapsed time starts at confirmation; HTTP runs in separate tasks.

Upload the matching `gas_sensor.ino` firmware as well. Both boards must share
the camera hotspot and permit client-to-client HTTP and mDNS. The reader resolves
`littersense-sensors` and uses the authenticated `/confirm-entry` endpoint.
The default pairing token is `LITTERSENSE_CONFIG_TOKEN`, which currently matches
the gas board's `SENSOR_CONFIG_TOKEN`. Override `LITTERSENSE_ULTRASONIC_TOKEN` in
the private RFID config if these diverge. For networks without mDNS, override
`LITTERSENSE_ULTRASONIC_HOST` with the sensor board's reserved IP, without a URL
scheme or path. Never use the PC or camera IP here. Missing sensor, invalid echo,
authentication failure, and late responses all fail closed (void entry).

## Configure and run

1. Save device provisioning settings in the webapp while signed into the owner account.
2. In `littersense_config.h`, fill in the 2.4 GHz Wi-Fi credentials, the deployed
   `https://YOUR-HOST/api/sensors` URL, the saved provisioning config token, and
   the endpoint's PEM root CA. For trusted local development only, set
   `LITTERSENSE_ALLOW_HTTP true` and use `http://192.168.68.108:3000/api/sensors`;
   no CA is needed for HTTP. Do not use a Firebase service-account key.
3. Register the exact EPC printed by this sketch in the cat's RFID tag field.
4. Build and upload for AI Thinker ESP32-CAM. This sketch is for a dedicated RFID
   board; uploading it replaces any camera firmware currently on that board.
5. Watch the Serial Monitor at 115200: `WIFI: connected, IP ...`, then `TIME: NTP
   synced`, then `SYNC: webapp reachable`. Any `WIFI: not connected` or `SYNC:
   pending, HTTP -1` line means the board never reached the PC.
6. Run the webapp and sign into that same owner account. Scan the registered tag,
   then obstruct the ultrasonic entrance zone within five seconds. Expect
   `PENDING ENTRY`, then `ENTRY` after confirmation. The entry should appear after
   a heartbeat (normally two seconds plus network latency). With an empty entrance,
   expect `VOID ENTRY` and no website entry. Remove the tag for three seconds to retry.
7. Remove the tag for at least three seconds of successful no-tag responses and
   scan it again. Check `SYNC: visit saved`, one visit in the dashboard, and the
   correct duration (10000 ms becomes 10 seconds).
8. Disconnect Wi-Fi, complete a session, reconnect, and verify it appears once.

Uploads use the existing `/api/sensors` ingestion and visit summaries. Entry
heartbeats do not count visits. Completed events retain their IDs until the server
acknowledges a recorded or duplicate visit. Unknown EPCs remain queued: register
the EPC under the device owner's account, then let it retry.

Live state is stored at `users/{ownerId}/deviceState/current`; signed-in dashboard
requests authenticate with Firebase ID tokens. One live device per owner is
supported by this existing dashboard shape. Legacy unauthenticated sensor GET
behavior remains for old clients; it does not receive this new scoped snapshot.

## Limits and verification

The offline queue holds 32 completed visits in RAM. Rebooting loses queued visits
and the active session; a full queue prints an error instead of silently dropping
an event. Keep Serial logs during testing. Add a flash journal before relying on
unattended operation across power failures. Wi-Fi and NTP must be reachable;
timestamps use network time, but durations always use the RFID millisecond clock.
Gas sensors and camera capture are not supplied by this RFID-only sketch.

## Camera and RFID boards together on local Wi-Fi

Keep the camera at `ESP32_STREAM_URL=http://192.168.68.120:81/stream`.
The RFID board posts to the PC; it does not expose a `/sensors` polling endpoint,
so do not replace `ESP32_SENSOR_URL` with its IP. No RFID environment variable
is needed. Set `NEXT_PUBLIC_DEVICE_CONFIG_ORIGIN=http://192.168.68.108:3000`
in the webapp's `.env.local`, then restart the app with
`npm.cmd run dev -- --hostname 0.0.0.0`. Reserve the PC's IP in the router.
Register the RFID EPC under the same account that owns the provisioning token.
The camera stream and RFID uploads use independent routes. A second board that
also posts sensor snapshots needs separate verification: the current shared
snapshot does not preserve active RFID state when another uploader omits it.

Host regression checks:

```powershell
g++ -std=c++11 -I tests tests/timer_test.cpp -o tests/timer_test.exe
./tests/timer_test.exe
g++ -std=c++11 -I tests tests/session_test.cpp -o tests/session_test.exe
./tests/session_test.exe
```

Webapp checks (from the webapp directory):

```powershell
node --test app/api/sensors/rfid.integration.test.mjs lib/utils/sensorSync.test.mjs lib/utils/deviceSensorSnapshot.test.cjs lib/hooks/useRfidVisitTracker.test.mjs
npx.cmd tsc --noEmit --pretty false
```

## Camera hotspot setup

Connection: RFID -> LitterSense hotspot -> CameraWebServer NAPT -> home router -> website PC.
The existing RFID SSID/password match the camera hotspot. Keep both passwords in sync when changing them.
The RFID uses DHCP: expect 192.168.4.x, gateway 192.168.4.1, and DNS 1.1.1.1 with the current camera configuration.
The camera must first report a home Wi-Fi connection and NAPT enabled. RFID uploads also wait for NTP.

The website is already configured with NEXT_PUBLIC_DEVICE_CONFIG_ORIGIN=http://192.168.68.108:3000.
From C:\Users\Admin\LitterSense-CSP-WebApp run:

```powershell
npm.cmd run dev -- --hostname 0.0.0.0
```

Keep LITTERSENSE_SENSOR_URL=http://192.168.68.108:3000/api/sensors while that PC IP remains assigned.
Allow the website process through Windows Firewall on the trusted private network if required.
Do not set ESP32_SENSOR_URL to the RFID IP: this board sends POSTs and has no polling server.
ESP32_STREAM_URL stays pointed at the camera's home-network address; verify that address from camera Serial.
The separate gas/ultrasonic board keeps its own URL.

Upload the RFID sketch to the confirmed dedicated RFID board, then check Serial at 115200:
WIFI connected -> expected gateway/DNS -> TIME: NTP synced -> heartbeat accepted.
Register the tag under the owner of the saved provisioning token, then scan/remove/scan.
Require SYNC: visit saved and a matching website visit before calling the integration verified.
From a phone on LitterSense with mobile data disabled, also verify public internet and camera video.
Compilation and mocked website tests do not verify NAPT, board upload, or real Firestore persistence.
