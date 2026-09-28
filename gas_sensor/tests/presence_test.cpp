#include <cassert>
#include <cmath>
#include <iostream>
#include "../ultrasonic_presence.h"

int main() {
    UltrasonicPresence sensor;
    sensor.sample(37, 100);
    sensor.request(100);
    sensor.sample(30, 200);
    assert(!sensor.accepted(200));
    sensor.sample(30, 300);
    assert(sensor.accepted(300) && sensor.occupied);
    sensor.sample(33, 400);
    assert(sensor.occupied); // Hysteresis band.
    sensor.sample(34, 500);
    assert(!sensor.occupied);
    assert(!sensor.accepted(5100));

    // Past presence never confirms a new request; require two fresh near samples.
    sensor.request(6000);
    sensor.sample(20, 6100);
    sensor.sample(20, 6200);
    assert(sensor.accepted(6200));
    sensor.request(6300);
    assert(!sensor.accepted(6300));
    sensor.sample(20, 6400);
    assert(!sensor.accepted(6400));
    sensor.sample(20, 6500);
    assert(sensor.accepted(6500));

    const float invalidSamples[] = {-1.0f, 0.0f, 1.9f, 30.1f, 37.0f, 401.0f, float(NAN)};
    for (float bad : invalidSamples) {
        sensor.request(7000);
        sensor.sample(20, 7100);
        sensor.sample(bad, 7200);
        sensor.sample(20, 7300);
        assert(!sensor.accepted(7300));
    }
    sensor.request(8000);
    sensor.sample(20, 8100);
    sensor.sample(20, 8500); // A long sampling pause cannot count as consecutive.
    assert(!sensor.accepted(8500));
    sensor.sample(20, 8600);
    assert(sensor.accepted(8600));

    sensor.request(UINT32_MAX - 100);
    sensor.sample(2, UINT32_MAX);
    sensor.sample(2, 99);
    assert(sensor.accepted(99));
    assert(!sensor.accepted(4899));
    sensor.request(10000);
    sensor.sample(20, 14900);
    sensor.sample(20, 15000);
    assert(!sensor.accepted(15000));
    std::cout << "PASS: thresholds, two fresh samples, noise/no echo, hysteresis, gaps, timeout, rollover\n";
}
