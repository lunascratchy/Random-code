#include "motor_driver.h"

MotorDriver::MotorDriver(uint8_t pinFwd, uint8_t pinRev,
                         uint8_t pinRearEn, uint8_t pinFrontEn,
                         uint8_t rearMinPwm, uint8_t frontMinPwm, float frontOffset)
    : _pinFwd(pinFwd), _pinRev(pinRev),
      _pinRearEn(pinRearEn), _pinFrontEn(pinFrontEn),
      _rearMin(rearMinPwm), _frontMin(frontMinPwm), _frontOffset(frontOffset) {}

void MotorDriver::begin() {
    pinMode(_pinFwd, OUTPUT);
    pinMode(_pinRev, OUTPUT);
    pinMode(_pinRearEn, OUTPUT);
    pinMode(_pinFrontEn, OUTPUT);
    stop();
}

void MotorDriver::stop() {
    analogWrite(_pinRearEn, 0);
    analogWrite(_pinFrontEn, 0);
    digitalWrite(_pinFwd, LOW);
    digitalWrite(_pinRev, LOW);
    _rearPwm = 0;
    _frontPwm = 0;
}

void MotorDriver::setEffort(int effort) {
    if (effort == 0) {
        stop();
        return;
    }

    bool forward = effort > 0;
    int mag = forward ? effort : -effort;
    if (mag > MOTOR_MAX_PWM) mag = MOTOR_MAX_PWM;
    // Below the stall PWM the motor just hums - bump to the floor.
    if (mag < _rearMin) mag = _rearMin;

    int front = (int)(mag * _frontOffset);
    if (front < _frontMin) front = _frontMin;
    if (front > MOTOR_MAX_PWM) front = MOTOR_MAX_PWM;

    digitalWrite(_pinFwd, forward ? HIGH : LOW);
    digitalWrite(_pinRev, forward ? LOW : HIGH);
    analogWrite(_pinRearEn, mag);
    analogWrite(_pinFrontEn, front);

    _rearPwm = (uint8_t)mag;
    _frontPwm = (uint8_t)front;
}
