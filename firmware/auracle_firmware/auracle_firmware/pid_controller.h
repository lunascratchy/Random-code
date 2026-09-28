#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"

// Incremental (velocity-form) PID, identical maths to the tuned teleop
// sketch: the output effort is *accumulated* each period, so pure Kp
// already drives steady-state error to zero. Effort is signed, which is
// what lets a side reverse for in-place turns.
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float deadzone);

    // target/measured in ticks/sec, dt in seconds. Returns signed effort
    // (-255..255). A target of 0 resets state and returns 0.
    int compute(float target, float measured, float dt);

    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _deadzone;
    float _integral = 0.0f;
    float _lastError = 0.0f;
    float _effort = 0.0f;   // kept as float so sub-1 PWM increments aren't truncated away
};

#endif
