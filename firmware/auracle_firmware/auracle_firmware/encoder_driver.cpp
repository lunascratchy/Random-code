#include "encoder_driver.h"

// Define the static instances
EncoderDriver* EncoderDriver::instanceL = nullptr;
EncoderDriver* EncoderDriver::instanceR = nullptr;

EncoderDriver::EncoderDriver(uint8_t pinA, uint8_t pinB) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    // Pre-calculate and cache raw AVR register pointers and bitmasks
    _portA = portInputRegister(digitalPinToPort(pinA));
    _bitmaskA = digitalPinToBitMask(pinA);

    _portB = portInputRegister(digitalPinToPort(pinB));
    _bitmaskB = digitalPinToBitMask(pinB);
}

void EncoderDriver::isrL() {
    // Quick compiler optimization shortcut: copy pointer to local register
    EncoderDriver* inst = instanceL; 
    if (!inst) return;

    // Ultra-fast direct AVR register read (replaces digitalRead)
    bool a = (*(inst->_portA) & inst->_bitmaskA);
    bool b = (*(inst->_portB) & inst->_bitmaskB);

    inst->_count += (a == b) ? 1 : -1;
}

void EncoderDriver::isrR() {
    EncoderDriver* inst = instanceR;
    if (!inst) return;

    // Ultra-fast direct AVR register read (replaces digitalRead)
    bool a = (*(inst->_portA) & inst->_bitmaskA);
    bool b = (*(inst->_portB) & inst->_bitmaskB);

    inst->_count += (a == b) ? 1 : -1;
}

long EncoderDriver::getCount() {
    // ATmega328P reads 32-bit longs in four 8-bit chunks. 
    // We must briefly pause interrupts to prevent data corruption.
    uint8_t oldSREG = SREG;
    noInterrupts();
    long value = _count;
    SREG = oldSREG; // Restores interrupt state slightly cleaner than interrupts()
    return value;
}

void EncoderDriver::reset() {
    uint8_t oldSREG = SREG;
    noInterrupts();
    _count = 0;
    SREG = oldSREG;
}
