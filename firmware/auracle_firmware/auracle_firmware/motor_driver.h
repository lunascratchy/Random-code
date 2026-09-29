#pragma once
#include <Arduino.h>

// One side: two motors sharing direction pins, separate PWM enables.
// Rear is closed-loop; front follows at rear PWM * frontOffset.
class MotorDriver {
public:
    MotorDriver(uint8_t pinFwd, uint8_t pinRev, uint8_t pinRearEn, uint8_t pinFrontEn,
                uint8_t rearMin, uint8_t frontMin, float frontOffset);
    void begin();
    // Signed effort -255..255. 0 = coast.
    void setEffort(int effort);
    void stop();

private:
    uint8_t pinFwd_, pinRev_, pinRearEn_, pinFrontEn_;
    uint8_t rearMin_, frontMin_;
    float frontOffset_;
};
