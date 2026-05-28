import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const here = dirname(fileURLToPath(import.meta.url));
const sketchRoot = join(here, "..", "..");
const appHttpd = readFileSync(join(sketchRoot, "app_httpd.cpp"), "utf8");
const litterSenseSketch = readFileSync(join(sketchRoot, "LitterSense-IoT.ino"), "utf8");

function functionBodyAfter(name, nextName) {
  const start = appHttpd.indexOf(name);
  assert.notEqual(start, -1, `${name} was not found`);

  const end = nextName ? appHttpd.indexOf(nextName, start) : appHttpd.length;
  assert.notEqual(end, -1, `${nextName} was not found after ${name}`);

  return appHttpd.slice(start, end);
}

function functionBodyFrom(source, name, nextName) {
  const start = source.indexOf(name);
  assert.notEqual(start, -1, `${name} was not found`);

  const end = nextName ? source.indexOf(nextName, start) : source.length;
  assert.notEqual(end, -1, `${nextName} was not found after ${name}`);

  return source.slice(start, end);
}

test("/sensors keeps the response buffer off the httpd task stack", () => {
  const sensorsHandler = functionBodyAfter(
    "static esp_err_t sensors_handler",
    "void startCameraServer",
  );

  assert.doesNotMatch(sensorsHandler, /char\s+json_response\s*\[\s*\d+\s*\]/);
  assert.match(sensorsHandler, /malloc\s*\(\s*SENSOR_JSON_RESPONSE_CAPACITY\s*\)/);
  assert.match(sensorsHandler, /free\s*\(\s*json_response\s*\)/);
});

test("camera routes are skipped when camera initialization failed", () => {
  const startCameraServer = functionBodyAfter("void startCameraServer", "void setupLedFlash");

  assert.match(startCameraServer, /extern\s+bool\s+cameraReady|cameraReady/);
  assert.match(startCameraServer, /if\s*\(\s*cameraReady\s*\)/);
  assert.match(startCameraServer, /if\s*\(\s*!\s*cameraReady\s*\)/);
  assert.match(startCameraServer, /httpd_register_uri_handler\s*\(\s*camera_httpd,\s*&sensors_uri\s*\)/);
});

test("camera initializes before GPIO 4 is reused for ultrasonic echo", () => {
  const setupBody = functionBodyFrom(litterSenseSketch, "void setup()", "void loop()");
  const sensorPinSetupBody = functionBodyFrom(
    litterSenseSketch,
    "void setupSensorPins()",
    "bool beginSdTransaction",
  );
  const cameraInit = setupBody.indexOf("cameraReady = startCamera();");
  const sensorPinSetup = setupBody.indexOf("setupSensorPins();");

  assert.ok(cameraInit >= 0, "camera init call was not found in setup()");
  assert.match(sensorPinSetupBody, /pinMode\s*\(\s*ULTRASONIC_ECHO_PIN\s*,\s*INPUT\s*\)/);
  assert.ok(sensorPinSetup >= 0, "sensor pin setup call was not found in setup()");
  assert.ok(
    cameraInit < sensorPinSetup,
    "sensor pins must not be configured until after camera init",
  );
});

test("offline logging uses flash storage instead of SD_MMC pins", () => {
  assert.doesNotMatch(litterSenseSketch, /#include\s+<SD_MMC\.h>/);
  assert.doesNotMatch(litterSenseSketch, /SD_MMC\./);
  assert.match(litterSenseSketch, /#include\s+<SPIFFS\.h>/);
  assert.match(litterSenseSketch, /const\s+bool\s+enableFlashOfflineLogging\s*=\s*true\s*;/);

  const beginSdTransaction = functionBodyFrom(
    litterSenseSketch,
    "bool beginSdTransaction",
    "void endSdTransaction",
  );
  const appendJsonLineToSd = functionBodyFrom(
    litterSenseSketch,
    "bool appendJsonLineToSd",
    "bool endsWithText",
  );
  const syncLoggedEventsBatch = functionBodyFrom(
    litterSenseSketch,
    "int syncLoggedEventsBatch",
    "bool configureWifiNetwork",
  );

  assert.match(beginSdTransaction, /if\s*\(\s*!\s*enableFlashOfflineLogging\s*\)/);
  assert.match(beginSdTransaction, /SPIFFS\.begin\s*\(\s*true\s*\)/);
  assert.doesNotMatch(beginSdTransaction, /rfid\.end\s*\(/);
  assert.doesNotMatch(beginSdTransaction, /pinMode\s*\(\s*ULTRASONIC_TRIG_PIN\s*,\s*INPUT\s*\)/);
  assert.match(beginSdTransaction, /return\s+false\s*;/);
  assert.match(appendJsonLineToSd, /if\s*\(\s*!\s*enableFlashOfflineLogging\s*\)/);
  assert.match(appendJsonLineToSd, /SPIFFS\.open\s*\(\s*path\s*,\s*FILE_APPEND\s*\)/);
  assert.match(syncLoggedEventsBatch, /if\s*\(\s*!\s*enableFlashOfflineLogging\s*\)[\s\S]*SENSOR_SYNC_NO_PENDING/);
  assert.match(syncLoggedEventsBatch, /SPIFFS\.open\s*\(\s*logPath\s*,\s*FILE_READ\s*\)/);
});

test("offline sync keeps large JSON buffers off the loop task stack", () => {
  assert.match(litterSenseSketch, /static\s+char\s+syncPostBody\s*\[\s*SYNC_BODY_MAX_LEN\s*\]/);
  assert.match(litterSenseSketch, /static\s+char\s+syncEventsJson\s*\[\s*SYNC_EVENTS_JSON_MAX_LEN\s*\]/);
  assert.match(litterSenseSketch, /static\s+char\s+syncEventLine\s*\[\s*SYNC_EVENT_LINE_MAX_LEN\s*\]/);

  const postEventBatch = functionBodyFrom(
    litterSenseSketch,
    "bool postEventBatch",
    "int syncLoggedEventsBatch",
  );
  const syncLoggedEventsBatch = functionBodyFrom(
    litterSenseSketch,
    "int syncLoggedEventsBatch",
    "bool configureWifiNetwork",
  );

  assert.doesNotMatch(postEventBatch, /char\s+body\s*\[\s*SYNC_BODY_MAX_LEN\s*\]/);
  assert.match(postEventBatch, /snprintf\s*\(\s*syncPostBody\s*,\s*sizeof\s*\(\s*syncPostBody\s*\)/);
  assert.match(postEventBatch, /http\.POST\s*\(\s*\(uint8_t\*\)\s*syncPostBody/);

  assert.doesNotMatch(syncLoggedEventsBatch, /char\s+eventsJson\s*\[\s*SYNC_EVENTS_JSON_MAX_LEN\s*\]/);
  assert.doesNotMatch(syncLoggedEventsBatch, /char\s+line\s*\[\s*SYNC_EVENT_LINE_MAX_LEN\s*\]/);
  assert.match(syncLoggedEventsBatch, /syncEventsJson\s*\[\s*0\s*\]\s*=\s*'\['/);
  assert.match(syncLoggedEventsBatch, /file\.readBytesUntil\s*\(\s*'\\n'\s*,\s*syncEventLine/);
  assert.match(syncLoggedEventsBatch, /postEventBatch\s*\(\s*syncEventsJson\s*,\s*eventCount\s*\)/);
});

test("RFID entry allows realistic confirmation but rejects very short false entries", () => {
  assert.match(litterSenseSketch, /#define\s+RFID_ENTRY_CONFIRMATION_WINDOW_MS\s+15000UL/);
  assert.match(litterSenseSketch, /#define\s+FALSE_ENTER_MAX_MS\s+8000UL/);
  assert.match(litterSenseSketch, /bool\s+pendingRfidEntry\s*=\s*false\s*;/);

  const updateRfidSessionLocked = functionBodyFrom(
    litterSenseSketch,
    "void updateRfidSessionLocked",
    "void printRfidTag",
  );
  const confirmPendingRfidEntry = functionBodyFrom(
    litterSenseSketch,
    "bool confirmPendingRfidEntryWithUltrasonicLocked",
    "void updateRfidSessionLocked",
  );
  const sensorTask = functionBodyFrom(litterSenseSketch, "void sensorTask", "");

  assert.match(updateRfidSessionLocked, /startPendingRfidEntryLocked\s*\(/);
  assert.doesNotMatch(updateRfidSessionLocked, /if\s*\(\s*!\s*rfidSessionActive\s*\)\s*\{[\s\S]*rfidSessionActive\s*=\s*true/);
  assert.match(confirmPendingRfidEntry, /RFID_ENTRY_CONFIRMATION_WINDOW_MS/);
  assert.match(confirmPendingRfidEntry, /rfidSessionActive\s*=\s*true/);
  assert.match(confirmPendingRfidEntry, /latestRfidEvent\s*=\s*"ENTER"/);
  assert.match(sensorTask, /expirePendingRfidEntryIfTimedOut\s*\(\s*now\s*\)/);
  assert.match(sensorTask, /confirmPendingRfidEntryWithUltrasonicLocked\s*\(/);
  assert.match(sensorTask, /if\s*\(\s*ultrasonicStateChanged\s*\)\s*\{[\s\S]*logUltrasonicEvent\s*\(\s*distanceMm\s*\)/);
});
