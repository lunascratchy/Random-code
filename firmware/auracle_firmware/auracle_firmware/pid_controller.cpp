#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float sample_time_s, float out_min, float out_max)
    : _out_min(out_min), _out_max(out_max), _sample_time_s(sample_time_s) {
    setTunings(kp, ki, kd); // Pre-calculate tunings with sample time immediately
}

float PIDController::compute(float target, float actual) {
    float error = target - actual;
    
    // Deadzone logic: don't wipe the integral, just ignore the error
    float absError = (error < 0.0f) ? -error : error;
    if (absError < PID_DEADZONE_TICKS_S) {
        error = 0.0f; // Treat error as zero, but let integral maintain holding force
    }

    // 1. Proportional term
    float p_term = _kp * error;

    // 2. Integral term (dt is already baked into _ki)
    float tentative_integral = _integral + (_ki * error);

    // 3. Derivative term (dt is already baked into _kd)
    float derivative = -(actual - _prev_actual); 
    float d_term = _kd * derivative;

    float output = p_term + tentative_integral + d_term;

    // Integral anti-windup clamping logic
    if (output > _out_max) {
        output = _out_max;
    } else if (output < _out_min) {
        output = _out_min;
    } else {
        // Only accumulate the integral error if the output isn't saturated
        // and we aren't in the deadzone (to prevent integral creeping during deadzone)
        if (error != 0.0f) {
            _integral = tentative_integral;
        }
    }

    // Update state for the next loop
    _prev_actual = actual; 
    
    // Deadzone final catch: if error is 0 and integral is 0, guarantee 0 output
    if (error == 0.0f && _integral == 0.0f) return 0.0f;
    
    return output;
}

void PIDController::reset() {
    _integral = 0.0f;
    _prev_actual = 0.0f;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    // Pre-multiply/divide by dt so we don't have to do it in the fast loop!
    _kp = kp;
    _ki = ki * _sample_time_s;
    _kd = kd / _sample_time_s;
}