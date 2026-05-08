import assert from "node:assert/strict";
import { mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { open, readdir, rm, stat } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const here = dirname(fileURLToPath(import.meta.url));
const sandboxRoot = join(here, ".tmp-sim");
const dataDir = join(sandboxRoot, "data");
const cursorPath = join(dataDir, "sync_cursor.txt");
const deviceId = "ESP32-CAM-TEST";
const dailyPath = join(dataDir, "2026-05-07.jsonl");
const batchMaxEvents = 8;

function resetSandbox() {
  if (!sandboxRoot.startsWith(here)) {
    throw new Error(`Refusing to delete outside simulation dir: ${sandboxRoot}`);
  }
  rmSync(sandboxRoot, { recursive: true, force: true });
  mkdirSync(dataDir, { recursive: true });
}

function appendJsonLine(line) {
  writeFileSync(dailyPath, `${line}\n`, { flag: "a" });
}

function logRfidEvent(tagHex, tagDec, eventName, timestamp) {
  appendJsonLine(
    JSON.stringify({
      type: "rfid",
      tagHex,
      tagDec,
      event: eventName,
      timestamp,
    }),
  );
}

function logGasEvent(mq135Raw, mq136Raw, timestamp) {
  appendJsonLine(
    JSON.stringify({
      type: "gas",
      mq135: { raw: mq135Raw, status: mq135Raw === 0 ? "GAS DETECTED" : "Clear" },
      mq136: { raw: mq136Raw, status: mq136Raw === 0 ? "GAS DETECTED" : "Clear" },
      timestamp,
    }),
  );
}

function logUltrasonicEvent(distanceMm, timestamp) {
  appendJsonLine(
    JSON.stringify({
      type: "ultrasonic",
      distanceCm: distanceMm / 10,
      timestamp,
    }),
  );
}

function logSessionEvent(tagHex, duration, status, timestamp) {
  appendJsonLine(
    JSON.stringify({
      type: "session",
      tagHex,
      duration,
      status,
      timestamp,
    }),
  );
}

function deriveSensorSyncUrl(configUrl) {
  const apiIndex = configUrl.indexOf("/api/");
  if (apiIndex <= 0) return "";

  const baseUrl = configUrl.slice(0, apiIndex);
  let configToken = configUrl.slice(configUrl.lastIndexOf("/") + 1);
  const queryIndex = configToken.indexOf("?");
  if (queryIndex >= 0) {
    configToken = configToken.slice(0, queryIndex);
  }

  return configToken
    ? `${baseUrl}/api/sensors?configToken=${configToken}`
    : `${baseUrl}/api/sensors`;
}

function readCursor() {
  try {
    const [path, offsetText] = readFileSync(cursorPath, "utf8").trim().split("|");
    return { path, offset: Number.parseInt(offsetText, 10) };
  } catch {
    return null;
  }
}

function writeCursor(path, offset) {
  writeFileSync(cursorPath, `${path}|${offset}\n`);
}

async function findNextLogFile() {
  const files = await readdir(dataDir);
  const jsonl = files.find((name) => name.endsWith(".jsonl"));
  return jsonl ? join(dataDir, jsonl) : null;
}

async function syncBatch(postOk, postedBodies) {
  let cursor = readCursor();
  let logPath = cursor?.path;
  let startOffset = cursor?.offset ?? 0;

  if (!logPath) {
    logPath = await findNextLogFile();
    startOffset = 0;
  }

  if (!logPath) {
    return "NO_PENDING";
  }

  const handle = await open(logPath, "r");
  const size = (await handle.stat()).size;
  if (startOffset > size) {
    startOffset = 0;
  }

  const buffer = Buffer.alloc(size - startOffset);
  await handle.read(buffer, 0, buffer.length, startOffset);
  await handle.close();

  const text = buffer.toString("utf8");
  const lines = text.split("\n").filter((line) => line.trim().length > 0);
  const batch = lines.slice(0, batchMaxEvents);
  if (batch.length === 0) {
    await rm(logPath, { force: true });
    await rm(cursorPath, { force: true });
    return "NO_PENDING";
  }

  const events = batch.map((line) => JSON.parse(line));
  const body = { deviceId, events };
  postedBodies.push(body);

  if (!postOk) {
    return "FAILED";
  }

  let endOffset = startOffset;
  for (const line of batch) {
    endOffset += Buffer.byteLength(`${line}\n`);
  }

  if (endOffset >= size) {
    await rm(logPath, { force: true });
    await rm(cursorPath, { force: true });
  } else {
    writeCursor(logPath, endOffset);
  }

  return "SUCCESS";
}

test("offline logging writes required event JSON lines", () => {
  resetSandbox();
  logRfidEvent("00967D97", "9862551", "ENTER", 1715100000);
  logGasEvent(1, 1, 1715100060);
  logUltrasonicEvent(153, 1715100005);
  logSessionEvent("00967D97", 150000, "NORMAL", 1715100150);

  const events = readFileSync(dailyPath, "utf8")
    .trim()
    .split("\n")
    .map((line) => JSON.parse(line));

  assert.deepEqual(
    events.map((event) => event.type),
    ["rfid", "gas", "ultrasonic", "session"],
  );
  assert.equal(events[0].tagHex, "00967D97");
  assert.equal(events[1].mq135.status, "Clear");
  assert.equal(events[2].distanceCm, 15.3);
  assert.equal(events[3].status, "NORMAL");
});

test("sensor sync URL keeps config token from device fetch URL", () => {
  assert.equal(
    deriveSensorSyncUrl("http://192.168.68.106:3000/api/device-config/cfg_abc1234567890"),
    "http://192.168.68.106:3000/api/sensors?configToken=cfg_abc1234567890",
  );
});

test("successful sync batches events and then deletes synced log file", async () => {
  resetSandbox();
  for (let index = 0; index < 10; index += 1) {
    logUltrasonicEvent(100 + index, 1715100000 + index);
  }

  const postedBodies = [];
  assert.equal(await syncBatch(true, postedBodies), "SUCCESS");
  assert.equal(postedBodies[0].deviceId, deviceId);
  assert.equal(postedBodies[0].events.length, 8);
  assert.ok(readCursor());

  assert.equal(await syncBatch(true, postedBodies), "SUCCESS");
  assert.equal(postedBodies[1].events.length, 2);
  await assert.rejects(stat(dailyPath));
  assert.equal(readCursor(), null);
});

test("failed sync keeps data and retries the same events", async () => {
  resetSandbox();
  for (let index = 0; index < 3; index += 1) {
    logRfidEvent(`TAG${index}`, `${index}`, "ENTER", 1715100100 + index);
  }

  const failedPosts = [];
  assert.equal(await syncBatch(false, failedPosts), "FAILED");
  assert.equal(failedPosts[0].events.length, 3);
  assert.equal(readCursor(), null);
  assert.ok(readFileSync(dailyPath, "utf8").includes("TAG0"));

  const retryPosts = [];
  assert.equal(await syncBatch(true, retryPosts), "SUCCESS");
  assert.deepEqual(
    retryPosts[0].events.map((event) => event.tagHex),
    ["TAG0", "TAG1", "TAG2"],
  );
  await assert.rejects(stat(dailyPath));
});
