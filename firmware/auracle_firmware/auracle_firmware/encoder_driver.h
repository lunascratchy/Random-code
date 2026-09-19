#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

class EncoderDriver {
public:
    EncoderDriver(uint8_t pinA, uint8_t pinB);
    long getCount();
    void reset();

    static EncoderDriver* instanceL;
    static EncoderDriver* instanceR;
    static void isrL();
    static void isrR();

private:
    volatile long _count = 0;

    // Fast AVR hardware register caching
    volatile uint8_t* _portA;
    volatile uint8_t* _portB;
    uint8_t _bitmaskA;
    uint8_t _bitmaskB;
};

#endif
