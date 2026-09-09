// motor_driver.cpp
#include "motor_driver.h"
MotorDriver::MotorDriver(int pwm, int in1, int in2) : _pwm(pwm), _in1(in1), _in2(in2) {
    pinMode(_pwm, OUTPUT); pinMode(_in1, OUTPUT); pinMode(_in2, OUTPUT);
}
void MotorDriver::setSpeed(double speed) {
    int pwmVal = constrain(abs(speed), 0, 255);
    digitalWrite(_in1, speed > 0 ? HIGH : LOW);
    digitalWrite(_in2, speed > 0 ? LOW : HIGH);
    analogWrite(_pwm, pwmVal);
}