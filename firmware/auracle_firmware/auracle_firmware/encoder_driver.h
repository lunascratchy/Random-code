#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H
#include <Arduino.h>

// Wraps ONE rear-wheel encoder. There are exactly two instances of this
// in the whole project (leftEnc, rightEnc) - the front wheels have no
// encoders at all (see config.h).
class EncoderDriver {
public:
    EncoderDriver(int pinA, int pinB);

    // Safe against interrupt tearing: a `long` is 4 bytes on AVR, so a
    // read can be corrupted if an ISR fires mid-read. This disables
    // interrupts for the few cycles it takes to copy the value out.
    long getCount();
    void reset();

    static EncoderDriver* instanceL;
    static EncoderDriver* instanceR;
    static void isrL();
    static void isrR();

private:
    int _pinA, _pinB;
    volatile long _count = 0;
};
#endif
