#pragma once
#include <Arduino.h>

// 1x decoding: call tick() from a RISING interrupt on the other channel.
class EncoderDriver {
public:
    EncoderDriver(uint8_t pinInt, uint8_t pinDir) : pinInt_(pinInt), pinDir_(pinDir) {}

    void begin() {
        pinMode(pinInt_, INPUT_PULLUP);
        pinMode(pinDir_, INPUT_PULLUP);
        dirPort_ = portInputRegister(digitalPinToPort(pinDir_));
        dirMask_ = digitalPinToBitMask(pinDir_);
    }

    // ISR: one port read instead of digitalRead().
    void tick() {
        if (*dirPort_ & dirMask_) count_++;
        else count_--;
    }

    long count() {
        noInterrupts();  // 32-bit read isn't atomic on AVR
        long c = count_;
        interrupts();
        return c;
    }

    uint8_t interruptPin() const { return pinInt_; }

private:
    uint8_t pinInt_, pinDir_;
    volatile uint8_t* dirPort_ = nullptr;
    uint8_t dirMask_ = 0;
    volatile long count_ = 0;
};
