#pragma once
#include <Arduino.h>
#include "config.h"

class PIDController {
public:
    int compute(float target, float measured, float dt) {
    if (target == 0.0f) { reset(); return 0; }

    float error = target - measured;
    if (fabs(error) < DEADZONE_TICKS_S) error = 0.0f;

    // velocity form: effort accumulates, so it can settle at whatever PWM the load needs
    effort_ += PID_KP * error;          // add Ki/Kd terms here if you start using them
    effort_ = constrain(effort_, -(float)MAX_PWM, (float)MAX_PWM);
    return (int)lroundf(effort_);
}

void reset() { effort_ = 0.0f; }

private:
    float integral_ = 0.0f;
    float lastError_ = 0.0f;
    float effort_ = 0.0f;
};