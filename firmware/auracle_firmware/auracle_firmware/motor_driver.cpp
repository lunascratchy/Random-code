#include "motor_driver.h"

MotorDriver::MotorDriver(uint8_t pwm, uint8_t in1, uint8_t in2) 
    : _pwm(pwm), _in1(in1), _in2(in2) {
    pinMode(_pwm, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
}

void MotorDriver::setSpeed(int speed) {
    if (speed == 0) {
        // Active Braking: clamp both directional pins HIGH (or both LOW)
        // and push PWM high to lock the motor electromagnetically.
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, HIGH);
        analogWrite(_pwm, 255); 
        return; // Skip the rest of the function
    }

    int absSpeed = (speed < 0) ? -speed : speed;
    int pwmVal = constrain(absSpeed, 0, MOTOR_MAX_PWM);
    
    if (pwmVal < MOTOR_MIN_PWM) {
        pwmVal = MOTOR_MIN_PWM;
    }

    // Direction control
    if (speed > 0) {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    }
    
    analogWrite(_pwm, pwmVal);
}