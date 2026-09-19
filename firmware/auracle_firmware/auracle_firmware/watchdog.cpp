#include "watchdog.h"

namespace {
uint8_t msToWdto(uint16_t timeout_ms) {
    // translator function to convert a millisecond timeout into the closest WDTO_ constant for wdt_enable().
    if (timeout_ms <= 15) return WDTO_15MS;
    if (timeout_ms <= 30) return WDTO_30MS;
    if (timeout_ms <= 60) return WDTO_60MS;
    if (timeout_ms <= 120) return WDTO_120MS;
    if (timeout_ms <= 250) return WDTO_250MS;
    if (timeout_ms <= 500) return WDTO_500MS;
    if (timeout_ms <= 1000) return WDTO_1S;
    if (timeout_ms <= 2000) return WDTO_2S;
    if (timeout_ms <= 4000) return WDTO_4S;
    return WDTO_8S;
}
}

void Watchdog::begin(uint16_t timeout_ms) {
    //starts timer, called in setup()
    wdt_enable(msToWdto(timeout_ms));
}

void Watchdog::pet() {
    //resets timer to zero as long as system is alive
    wdt_reset();
}

bool Watchdog::isCommsAlive(unsigned long lastPacketTime, unsigned long timeout) {
    //compares the time since the last packet was received to the timeout value, returns true if the comms are alive, false if not
    return (millis() - lastPacketTime) <= timeout;
}
