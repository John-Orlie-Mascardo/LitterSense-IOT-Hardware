#include <Arduino.h>
#include "littersense_sync.h"
#include "rfid_confirmation.h"

// AI-Thinker ESP32-CAM wiring:
// RFID TXD -> GPIO13
// RFID RXD -> GPIO14
// RFID EN  -> GPIO15
// RFID VCC -> 5V
// RFID GND -> GND

const int PIN_RFID_RX = 13;
const int PIN_RFID_TX = 14;
const int PIN_RFID_EN = 15;

HardwareSerial rfid(2);

const uint32_t POLL_INTERVAL_MS = 200;
const uint32_t REMOVE_TAG_MS = 3000;
const uint32_t REPLY_TIMEOUT_MS = 300;
bool awaitingReply = false;
bool powerPending = true;
uint32_t commandSentAt = 0;
// Persistent UART assembly: retain partial frames between loop calls.
uint8_t frame[256];
size_t used = 0;
size_t expected = 0;

// Request 10 dBm (1000 = 0x03E8); checksum = 0xA3.
// This DOES NOT guarantee a 10 cm reading range.
const uint8_t CMD_POWER_10DBM[] = {
  0xBB, 0x00, 0xB6, 0x00, 0x02,
  0x03, 0xE8, 0xA3, 0x7E
};

const uint8_t CMD_POLL_ONCE[] = {
  0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E
};

// Session state: remains inside even after the tag is removed.
bool inside = false;
String activeTag;
uint32_t entryTime = 0;
String pendingTag;
uint32_t pendingAt = 0;
uint32_t pendingRequest = 0;

// Scan latch: prevents repeated reads from becoming new events.
bool scanArmed = true;
bool clearWindowStarted = false;
uint32_t clearWindowStart = 0;
uint32_t lastPollTime = 0;

void resetClearWindow() {
  clearWindowStarted = false;
}

void sendCommand(const uint8_t *data, size_t length) {
  rfid.write(data, length);
}

// Non-blocking frame reader: length, -1 invalid, or -2 incomplete.
int readFrame(
  uint8_t &type,
  uint8_t &command,
  uint8_t *payload,
  size_t capacity
) {


  size_t budget = sizeof(frame);
  while (rfid.available() && budget-- > 0) {

    uint8_t value = (uint8_t)rfid.read();

    if (used == 0 && value != 0xBB) {
      continue;
    }

    frame[used++] = value;

    if (used == 5) {
      uint16_t length =
        ((uint16_t)frame[3] << 8) | frame[4];

      expected = (size_t)length + 7;

      if (length > capacity || expected > sizeof(frame)) {
        used = expected = 0;
        return -1;
      }
    }

    if (expected != 0 && used == expected) {
      if (frame[expected - 1] != 0x7E) {
        used = expected = 0;
        return -1;
      }

      uint8_t checksum = 0;

      for (size_t i = 1; i < expected - 2; i++) {
        checksum += frame[i];
      }

      if (checksum != frame[expected - 2]) {
        used = expected = 0;
        return -1;
      }

      type = frame[1];
      command = frame[2];

      size_t length = expected - 7;
      memcpy(payload, frame + 5, length);

      used = expected = 0;
      return (int)length;
    }
  }

  return -2; // Incomplete: return immediately and retain bytes.
}

String extractEPC(const uint8_t *payload, int length) {
  // RSSI(1) + PC(2) + EPC(variable) + CRC(2)
  if (length < 7) {
    return "";
  }

  size_t epcLength = (size_t)length - 5;

  // PC's upper five bits specify EPC length in 16-bit words.
  if (epcLength != (size_t)(payload[1] >> 3) * 2) {
    return "";
  }
  const char hex[] = "0123456789ABCDEF";
  String epc;
  for (size_t i = 0; i < epcLength; ++i) {
    epc += hex[payload[i + 3] >> 4];
    epc += hex[payload[i + 3] & 0x0F];
  }
  return epc;
}

void handleTag(const String &tag, uint32_t now) {
  resetClearWindow();
  if (pendingTag.length() || !scanArmed || (inside && tag != activeTag)) return;
  scanArmed = false;
  if (!inside) {
    pendingTag = tag;
    pendingAt = now;
    pendingRequest = requestUltrasonicConfirmation(now);
    Serial.print("PENDING ENTRY: ");
    Serial.println(tag);
  } else {
    inside = false;
    publishRfidState(tag, entryTime, now, false);
    Serial.print("EXIT: ");
    Serial.print(tag);
    Serial.print(" duration (ms): ");
    Serial.println((uint32_t)(now - entryTime));
    activeTag = "";
  }
}

void handlePendingEntry(uint32_t now) {
  if (!pendingTag.length()) return;
  // Deadline wins over any late or delayed network response, including millis rollover.
  if (uint32_t(now - pendingAt) >= RFID_CONFIRM_TIMEOUT_MS) {
    Serial.print("VOID ENTRY (no ultrasonic confirmation): ");
    Serial.println(pendingTag);
  } else if (ultrasonicConfirmed(pendingRequest)) {
    inside = true;
    activeTag = pendingTag;
    entryTime = now;
    publishRfidState(activeTag, entryTime, now, true);
    Serial.print("ENTRY: ");
    Serial.println(activeTag);
  } else {
    return;
  }
  pendingTag = "";
  scanArmed = false;
  resetClearWindow(); // Require a fresh removal window before the next scan.
}

void handleNoTag(uint32_t now) {
  if (!clearWindowStarted) {
    clearWindowStarted = true;
    clearWindowStart = now;
  }
  if ((uint32_t)(now - clearWindowStart) >= REMOVE_TAG_MS) {
    scanArmed = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RFID_EN, OUTPUT);
  digitalWrite(PIN_RFID_EN, HIGH);
  rfid.begin(115200, SERIAL_8N1, PIN_RFID_RX, PIN_RFID_TX);
  delay(1000);
  beginRfidSync();
  beginUltrasonicConfirmation();
}

void loop() {
  const uint32_t now = millis();
  handlePendingEntry(now);
  uint8_t type = 0, command = 0, payload[249];
  const int length = readFrame(type, command, payload, sizeof(payload));
  if (length == -1) {
    resetClearWindow();
  } else if (length >= 0 && awaitingReply) {
    if (powerPending && type == 0x01 && command == 0xB6) {
      powerPending = false;
      awaitingReply = false;
    } else if (!powerPending && type == 0x02 && command == 0x22) {
      awaitingReply = false;
      String tag = extractEPC(payload, length);
      if (tag.length() != 0) handleTag(tag, now);
      else resetClearWindow();
    } else if (type == 0x01 && command == 0xFF) {
      awaitingReply = false;
      if (!powerPending && length == 1 && payload[0] == 0x15) {
        handleNoTag(now);
      } else {
        resetClearWindow();
      }
    }
  }
  if (awaitingReply && (uint32_t)(now - commandSentAt) >= REPLY_TIMEOUT_MS) {
    awaitingReply = false;
    used = expected = 0;
    resetClearWindow();
  }
  if (!awaitingReply && (uint32_t)(now - lastPollTime) >= POLL_INTERVAL_MS) {
    if (powerPending) sendCommand(CMD_POWER_10DBM, sizeof(CMD_POWER_10DBM));
    else sendCommand(CMD_POLL_ONCE, sizeof(CMD_POLL_ONCE));
    awaitingReply = true;
    commandSentAt = lastPollTime = now;
  }
}
