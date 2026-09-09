#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include <avr/wdt.h> // Standard AVR library for Nano/Uno

class Watchdog {
public:
    // Starts the hardware timer (timeout in milliseconds)
    void begin(uint8_t timeout_ms = 2000);
    
    // "Pets" the dog to prevent reset
    void pet();

    // Communication Watchdog: checks if we've heard from ROS recently
    bool isCommsAlive(unsigned long lastPacketTime, unsigned long timeout = 500);
};

#endif