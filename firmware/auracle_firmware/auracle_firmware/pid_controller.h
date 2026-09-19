#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float sample_time_s, float out_min = -255.0f, float out_max = 255.0f);

    float compute(float target, float actual);

    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _out_min, _out_max;
    float _integral = 0.0f;
    float _prev_actual = 0.0f; 
    float _sample_time_s;
};

#endif