#include <cassert>
#include <iostream>
#include "../littersense_rfid_test.ino"

void replyAt(uint32_t now, bool tag) {
  clockMs = now;
  awaitingReply = true;
  commandSentAt = now - 20;
  if (tag) {
    rfid.rx = {0xBB, 0x02, 0x22, 0, 7, 0, 8, 0, 1, 0xAB, 0, 0, 0xDF, 0x7E};
  } else {
    rfid.rx = {0xBB, 1, 0xFF, 0, 1, 0x15, 0x16, 0x7E};
  }
  loop();
}

int main() {
  const uint32_t starts[] = {1000, UINT32_MAX - 2000};
  for (uint32_t start : starts) {
    for (uint32_t duration : {5000u, 10000u, 60000u}) {
      inside = false;
      pendingTag = "";
      scanArmed = true;
      powerPending = false;
      used = expected = 0;
      resetClearWindow();
      Serial.output.str("");
      replyAt(start, true);
      assert(!inside && pendingTag == "01AB");
      testConfirmedGeneration = pendingRequest;
      handlePendingEntry(start + 100);
      assert(inside && entryTime == start + 100);
      replyAt(start + 200, true);
      assert(inside && entryTime == start + 100);
      for (uint32_t offset = 400; offset < duration; offset += 200) {
        replyAt(start + offset, false);
        assert(inside);
      }
      replyAt(start + duration, true);
      assert(!inside);
      const String expectedOutput = "PENDING ENTRY: 01AB\nENTRY: 01AB\nEXIT: 01AB duration (ms): "
        + std::to_string(duration - 100) + "\n";
      assert(Serial.output.str() == expectedOutput);
      std::cout << "PASS: " << duration / 1000 << " simulated seconds -> "
                << duration - 100 << " ms since confirmation; start=" << start << '\n';
    }
  }
}
