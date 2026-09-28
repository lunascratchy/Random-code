#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float deadzone)
    : _kp(kp), _ki(ki), _kd(kd), _deadzone(deadzone) {}

int PIDController::compute(float target, float measured, float dt) {
    if (target == 0.0f) {
        // Clean stop, and don't carry wound-up state into the next command.
        reset();
        return 0;
    }

    float error = target - measured;
    if (fabs(error) < _deadzone) {
        error = 0.0f;
    }

    float derivative = (error - _lastError) / dt;
    _lastError = error;

    // Anti-windup: only integrate while the output isn't pinned.
    bool saturated = (_effort >= MOTOR_MAX_PWM && error > 0) ||
                     (_effort <= -MOTOR_MAX_PWM && error < 0);
    if (!saturated) {
        _integral += error * dt;
    }

    _effort += _kp * error + _ki * _integral + _kd * derivative;
    _effort = constrain(_effort, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);

    return (int)lroundf(_effort);
}

void PIDController::reset() {
    _integral = 0.0f;
    _lastError = 0.0f;
    _effort = 0.0f;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
