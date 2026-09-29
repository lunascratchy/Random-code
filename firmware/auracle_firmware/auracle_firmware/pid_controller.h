#pragma once
#include <Arduino.h>
#include "config.h"

// Incremental PID: effort accumulates each period, so pure Kp already
// removes steady-state error. Signed output lets a side reverse.
class PIDController {
public:
    // target/measured in ticks/s, dt in s. Returns effort -255..255.
    int compute(float target, float measured, float dt) {
        if (target == 0.0f) {
            reset();
            return 0;
        }
        float error = target - measured;
        if (fabs(error) < DEADZONE_TICKS_S) error = 0.0f;

        integral_ += error * dt;
        float derivative = (error - lastError_) / dt;
        lastError_ = error;

        effort_ += PID_KP * error + PID_KI * integral_ + PID_KD * derivative;
        effort_ = constrain(effort_, -(float)MAX_PWM, (float)MAX_PWM);
        return (int)lroundf(effort_);
    }

    void reset() { integral_ = lastError_ = effort_ = 0.0f; }

private:
    float integral_ = 0.0f;
    float lastError_ = 0.0f;
    float effort_ = 0.0f;
};
