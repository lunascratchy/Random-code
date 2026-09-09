// pid_controller.cpp
#include "pid_controller.h"
PIDController::PIDController(double kp, double ki, double kd) : _kp(kp), _ki(ki), _kd(kd) {}
double PIDController::compute(double target, double actual, double dt) {
    double error = target - actual;
    _integral += error * dt;
    double derivative = (error - _prev_error) / dt;
    _prev_error = error;
    return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}