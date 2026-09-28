#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

// 1x quadrature decoding: RISING edge on the interrupt channel, the
// other channel's level gives direction. Matches the bench tests, so
// the tuned Kp and measured ticks/sec carry over unchanged.
class EncoderDriver {
public:
    EncoderDriver(uint8_t pinInt, uint8_t pinDir);
    void begin();
    long getCount();
    void reset();

    static EncoderDriver* instanceL;
    static EncoderDriver* instanceR;
    static void isrL();
    static void isrR();

private:
    uint8_t _pinInt;
    uint8_t _pinDir;
    volatile long _count = 0;

    // Cached AVR input register + mask for the direction pin, so the
    // ISR is a single port read instead of digitalRead().
    volatile uint8_t* _dirPort;
    uint8_t _dirMask;
};

#endif
