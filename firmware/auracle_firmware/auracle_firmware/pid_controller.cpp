#include "pid_controller.h"
#include <math.h>

PIDController::PIDController(double kp, double ki, double kd, double out_min, double out_max)
    : _kp(kp), _ki(ki), _kd(kd), _out_min(out_min), _out_max(out_max) {}

double PIDController::compute(double target, double actual, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }

    double error = target - actual;

    // Deadzone: near a held/zero target, tiny encoder jitter produces a
    // small nonzero error every cycle. Left alone, that gets amplified
    // by motor_driver's deadband compensation into a real, audible PWM
    // pulse - constant buzzing/chatter even when the robot should be
    // sitting still. Threshold is in ticks/sec (this loop's real unit),
    // not raw ticks-per-fixed-period, so it stays meaningful even if
    // CONTROL_PERIOD_MS changes later. See PID_DEADZONE_TICKS_S in
    // config.h to tune it against your own encoder noise floor.
    if (fabs(error) < PID_DEADZONE_TICKS_S) {
        _integral = 0;
        _prev_error = error;
        return 0.0;
    }

    // Conditional integration (anti-windup): only commit the integral
    // update if doing so doesn't push output past the clamp. Otherwise
    // the integral term keeps growing while the motor is already
    // saturated, causing overshoot once the error direction flips.
    double tentative_integral = _integral + error * dt;
    double derivative = (error - _prev_error) / dt;
    double output = (_kp * error) + (_ki * tentative_integral) + (_kd * derivative);

    if (output > _out_max) {
        output = _out_max;
    } else if (output < _out_min) {
        output = _out_min;
    } else {
        _integral = tentative_integral;
    }

    _prev_error = error;
    return output;
}

void PIDController::reset() {
    _integral = 0;
    _prev_error = 0;
}

void PIDController::setTunings(double kp, double ki, double kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
