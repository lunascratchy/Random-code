#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
    // Uses 8-bit integers for pin storage to save precious SRAM
    MotorDriver(uint8_t pwm, uint8_t in1, uint8_t in2);
    
    // Accepts an integer speed (e.g., matching your PWM or integer control bounds)
    void setSpeed(int speed);

private:
    uint8_t _pwm;
    uint8_t _in1;
    uint8_t _in2;
};

#endif
