#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// --- Motor driver pins (one L298N per side) ---
#define LEFT_PWM   5
#define LEFT_IN1   9
#define LEFT_IN2   10
#define RIGHT_PWM  6
#define RIGHT_IN1  7
#define RIGHT_IN2  8
// --- Encoder pins
#define LEFT_ENC_A   2   // interrupt pin
#define LEFT_ENC_B   4
#define RIGHT_ENC_A  3   // interrupt pin
#define RIGHT_ENC_B  12
// --- Servo motor
#define SERVO_PIN 11
// MPU6050 IMU: SDA = A4, SCL = A5 (I2C, fixed pins, nothing to #define)
// --- Serial baudrate ---
#define BAUDRATE 115200
// --- Encoder scaling ---
#define LEFT_TICKS_PER_REV 730.0
#define RIGHT_TICKS_PER_REV 655.0
#define RAD_S_TO_TICKS_S_L(rad_s) ((rad_s) * LEFT_TICKS_PER_REV / (2.0 * PI))
#define RAD_S_TO_TICKS_S_R(rad_s) ((rad_s) * RIGHT_TICKS_PER_REV / (2.0 * PI))
// --- Motor PWM ---
#define MOTOR_MAX_PWM 255
#define MOTOR_MIN_PWM 20
// --- PID ---
#define PID_KP 1.5
#define PID_KI 0.1
#define PID_KD 0.01
#define CONTROL_PERIOD_MS 33   // ~30Hz, matches a typical ros2_control update_rate
#define SAMPLE_TIME_S 0.033f   // Needed for the optimized PID constructor
// Error smaller than this (ticks/sec) is treated as "at target" and snapped to zero output, to stop standstill motor buzz from encoder jitter
#define PID_DEADZONE_TICKS_S 5.0
// How often the PID/motor/telemetry block runs, in milliseconds.
#define CONTROL_PERIOD_MS 33   // ~30Hz, matches a typical ros2_control update_rate
// --Watchdog timeouts (ms)---
#define WDT_TIMEOUT_MS 2000
#define COMMS_TIMEOUT_MS 500

#endif
