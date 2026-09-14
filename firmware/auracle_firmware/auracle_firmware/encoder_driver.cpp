#include "encoder_driver.h"

EncoderDriver* EncoderDriver::instanceL = nullptr;
EncoderDriver* EncoderDriver::instanceR = nullptr;

EncoderDriver::EncoderDriver(int pinA, int pinB) : _pinA(pinA), _pinB(pinB) {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
}

// Attach these with mode CHANGE (both edges of channel A), not RISING -
// this doubles the effective resolution for the same physical encoder
// disc compared to counting rising edges only.
//
// SIGN CONVENTION: both ISRs below use the same rule (A==B -> +1). This
// is a starting assumption, not a guarantee for your wiring. VERIFY ON
// THE BENCH before trusting it: spin each rear wheel FORWARD by hand
// and confirm getCount() increases on both sides. If one side counts
// backwards, it's almost always because that motor/encoder is
// physically mirror-mounted relative to the other - fix it by flipping
// that ISR's sign here, not by guessing or "fixing" it in software
// somewhere else downstream.
void EncoderDriver::isrL() {
    if (!instanceL) return;
    bool a = digitalRead(instanceL->_pinA);
    bool b = digitalRead(instanceL->_pinB);
    instanceL->_count += (a == b) ? 1 : -1;
}

void EncoderDriver::isrR() {
    if (!instanceR) return;
    bool a = digitalRead(instanceR->_pinA);
    bool b = digitalRead(instanceR->_pinB);
    instanceR->_count += (a == b) ? 1 : -1;
}

long EncoderDriver::getCount() {
    noInterrupts();
    long value = _count;
    interrupts();
    return value;
}

void EncoderDriver::reset() {
    noInterrupts();
    _count = 0;
    interrupts();
}
