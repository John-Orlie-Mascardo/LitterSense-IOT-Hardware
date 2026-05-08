import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const here = dirname(fileURLToPath(import.meta.url));
const sketchRoot = join(here, "..", "..");
const sketch = readFileSync(join(sketchRoot, "LitterSense-IoT.ino"), "utf8");

function functionBodyFrom(source, name, nextName) {
  const start = source.indexOf(name);
  assert.notEqual(start, -1, `${name} was not found`);

  const end = nextName ? source.indexOf(nextName, start) : source.length;
  assert.notEqual(end, -1, `${nextName} was not found after ${name}`);

  return source.slice(start, end);
}

test("runtime reconnect retries saved Wi-Fi without reopening provisioning", () => {
  const reconnectHelper = functionBodyFrom(
    sketch,
    "bool reconnectSavedWifiOnly",
    "uint8_t hexPairToByte",
  );
  const loopBody = functionBodyFrom(sketch, "void loop()", "void sensorTask");

  assert.match(reconnectHelper, /connectWifi\s*\(\s*savedSsid\s*,\s*savedPassword/);
  assert.doesNotMatch(reconnectHelper, /runSetupPortal\s*\(/);

  const disconnectedBranch = loopBody.slice(
    loopBody.indexOf("if (WiFi.status() != WL_CONNECTED)"),
    loopBody.indexOf("} else {", loopBody.indexOf("if (WiFi.status() != WL_CONNECTED)")),
  );

  assert.match(disconnectedBranch, /reconnectSavedWifiOnly\s*\(/);
  assert.doesNotMatch(disconnectedBranch, /connectWithStoredOrProvisionedWifi\s*\(/);
  assert.doesNotMatch(disconnectedBranch, /runSetupPortal\s*\(/);
});
