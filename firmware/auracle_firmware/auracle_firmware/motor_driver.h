#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "config.h"

// One side of the robot: two motors sharing bridged direction pins
// (IN1+IN3 fwd, IN2+IN4 rev) but with independent PWM enables.
// The rear motor is the closed-loop one; the front follows it open-loop
// at rearPwm * frontOffset.
class MotorDriver {
public:
    MotorDriver(uint8_t pinFwd, uint8_t pinRev,
                uint8_t pinRearEn, uint8_t pinFrontEn,
                uint8_t rearMinPwm, uint8_t frontMinPwm, float frontOffset);

    void begin();

    // Signed rear effort, -255..255. Sign = direction, magnitude = PWM.
    // 0 = coast (both direction pins LOW, both PWM 0).
    void setEffort(int effort);
    void stop();

    uint8_t rearPwm() const { return _rearPwm; }
    uint8_t frontPwm() const { return _frontPwm; }

private:
    uint8_t _pinFwd, _pinRev, _pinRearEn, _pinFrontEn;
    uint8_t _rearMin, _frontMin;
    float _frontOffset;
    uint8_t _rearPwm = 0, _frontPwm = 0;
};

#endif
