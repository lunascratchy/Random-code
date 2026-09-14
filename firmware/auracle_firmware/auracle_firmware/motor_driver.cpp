#include "motor_driver.h"
#include <math.h>

MotorDriver::MotorDriver(int pwm, int in1, int in2) : _pwm(pwm), _in1(in1), _in2(in2) {
    pinMode(_pwm, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
}

void MotorDriver::setSpeed(double speed) {
    // fabs(), not abs() - Arduino's abs() is an int macro and silently
    // truncates a double to int before taking the absolute value, which
    // is wrong for anything between -1.0 and 1.0.
    int pwmVal = (int)constrain(fabs(speed), 0, MOTOR_MAX_PWM);

    // Deadband compensation: below MOTOR_MIN_PWM the motor draws current
    // but doesn't actually turn, which stalls the PID loop (it sees zero
    // ticks of motion and keeps ramping error/integral trying to "push
    // harder" on a wheel that was never going to move at that PWM).
    // Bump any nonzero command up to the measured minimum that moves it.
    if (pwmVal > 0 && pwmVal < MOTOR_MIN_PWM) {
        pwmVal = MOTOR_MIN_PWM;
    }

    digitalWrite(_in1, speed > 0 ? HIGH : LOW);
    digitalWrite(_in2, speed > 0 ? LOW : HIGH);
    analogWrite(_pwm, pwmVal);
}
