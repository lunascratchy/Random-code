#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include <Arduino.h>
#include "config.h"

// One instance per SIDE (left/right), not per motor - the two physical
// motors on a side share the same L298N output and always move together.
class MotorDriver {
public:
    MotorDriver(int pwm, int in1, int in2);

    // speed: -255..255 (PID output units). Handles direction, deadband
    // compensation, and clamping internally.
    void setSpeed(double speed);

private:
    int _pwm, _in1, _in2;
};
#endif
