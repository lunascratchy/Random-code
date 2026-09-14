#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include <avr/wdt.h>
#include "config.h"

// Two DIFFERENT kinds of "watchdog" living in one small class, on
// purpose - they catch two different failure modes and neither one
// substitutes for the other:
//   - The hardware WDT (begin/pet) resets the whole chip if loop()
//     itself ever hangs (e.g. an I2C bus lockup on the IMU).
//   - isCommsAlive() only checks whether the Pi has stopped talking to
//     us - the chip is fine, but nobody's driving anymore, so stop.
class Watchdog {
public:
    // timeout_ms is rounded UP to the nearest AVR WDTO_ setting.
    // uint16_t, not uint8_t - a uint8_t silently truncates anything
    // above 255 (an intended 2000ms would become 2000 % 256 = 208ms).
    void begin(uint16_t timeout_ms = WDT_TIMEOUT_MS);

    // Pets the hardware watchdog. Call this first thing, every loop(),
    // unconditionally - before any logic that could conceivably stall.
    void pet();

    // Communication watchdog: true if we've heard a 'v' command from
    // the Pi within `timeout` ms.
    bool isCommsAlive(unsigned long lastPacketTime, unsigned long timeout = COMMS_TIMEOUT_MS);
};

#endif
