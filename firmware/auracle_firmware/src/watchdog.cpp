#include "watchdog.h"

void Watchdog::begin(uint8_t timeout_ms) {
    // Enable the hardware watchdog timer
    wdt_enable(WDTO_2S); // Nano only supports specific steps, 2s is standard
}

void Watchdog::pet() {
    // Reset the hardware timer
    wdt_reset();
}

bool Watchdog::isCommsAlive(unsigned long lastPacketTime, unsigned long timeout) {
    // Check if current time minus last packet time is greater than timeout
    if (millis() - lastPacketTime > timeout) {
        return false; // Comms are dead!
    }
    return true; // Everything is fine
}