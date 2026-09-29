#pragma once
#include <Arduino.h>
#include "config.h"

class PIDController {
public:
    int compute(float target, float measured, float dt) {
        if (target == 0.0f) {
            reset();
            return 0;
        }
        float error = target - measured;
        if (fabs(error) < DEADZONE_TICKS_S) error = 0.0f;

        integral_ += error * dt;
        
        // Anti-windup guard for the integral state
        float max_integral = MAX_PWM / (PID_KI > 0 ? PID_KI : 1.0f);
        integral_ = constrain(integral_, -max_integral, max_integral);

        float derivative = (error - lastError_) / dt;
        lastError_ = error;

        // Calculate absolute effort directly (removed +=)
        effort_ = PID_KP * error + PID_KI * integral_ + PID_KD * derivative;
        effort_ = constrain(effort_, -(float)MAX_PWM, (float)MAX_PWM);
        
        return (int)lroundf(effort_);
    }

    void reset() { integral_ = lastError_ = effort_ = 0.0f; }

private:
    float integral_ = 0.0f;
    float lastError_ = 0.0f;
    float effort_ = 0.0f;
};