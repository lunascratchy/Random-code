#ifndef CONFIG_H
#define CONFIG_H

// --- Motor Pins (L298N) ---
#define L_MOTOR_PWM 5
#define L_MOTOR_IN1 4
#define L_MOTOR_IN2 3
#define R_MOTOR_PWM 6
#define R_MOTOR_IN1 7
#define R_MOTOR_IN2 8

// --- Encoder Pins ---
#define L_ENC_A 2  // Interrupt pin
#define L_ENC_B 10
#define R_ENC_A 3  // Interrupt pin
#define R_ENC_B 11

// --- Constants ---
#define BAUDRATE 115200
#define PPR 3436.0        // Pulses per revolution
#define WHEEL_RADIUS 0.0239
#define PID_KP 1.5
#define PID_KI 0.1
#define PID_KD 0.01

#endif