#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include <avr/wdt.h>
#include "config.h"

class Watchdog {
public:
    void begin(uint16_t timeout_ms = WDT_TIMEOUT_MS);
    void pet();
    bool isCommsAlive(unsigned long lastPacketTime, unsigned long timeout = COMMS_TIMEOUT_MS);
};

#endif
