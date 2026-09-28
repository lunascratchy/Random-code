#include "encoder_driver.h"

EncoderDriver* EncoderDriver::instanceL = nullptr;
EncoderDriver* EncoderDriver::instanceR = nullptr;

EncoderDriver::EncoderDriver(uint8_t pinInt, uint8_t pinDir)
    : _pinInt(pinInt), _pinDir(pinDir) {}

void EncoderDriver::begin() {
    pinMode(_pinInt, INPUT_PULLUP);
    pinMode(_pinDir, INPUT_PULLUP);
    _dirPort = portInputRegister(digitalPinToPort(_pinDir));
    _dirMask = digitalPinToBitMask(_pinDir);
}

// Direction pin HIGH on the rising edge = forward = count up
// (same convention as readLeftEncoder/readRightEncoder in the bench tests).
void EncoderDriver::isrL() {
    EncoderDriver* inst = instanceL;
    if (*(inst->_dirPort) & inst->_dirMask) inst->_count++;
    else inst->_count--;
}

void EncoderDriver::isrR() {
    EncoderDriver* inst = instanceR;
    if (*(inst->_dirPort) & inst->_dirMask) inst->_count++;
    else inst->_count--;
}

long EncoderDriver::getCount() {
    // 32-bit read is 4 bytes on AVR - pause interrupts so it can't tear.
    uint8_t oldSREG = SREG;
    noInterrupts();
    long value = _count;
    SREG = oldSREG;
    return value;
}

void EncoderDriver::reset() {
    uint8_t oldSREG = SREG;
    noInterrupts();
    _count = 0;
    SREG = oldSREG;
}
