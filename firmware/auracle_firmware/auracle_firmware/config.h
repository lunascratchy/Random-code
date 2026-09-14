#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// BOARD: Arduino Nano (ATmega328P)
//
// DRIVETRAIN LAYOUT: 6 wheels total, 4 motorized (front-left,
// front-right, rear-left, rear-right), 2 passive/idle (middle).
// The two motors on each SIDE are wired to the SAME L298N output in
// parallel - they are not independently controllable. Electrically
// this is a 2-channel (left/right) differential drive, even though it
// physically moves 4 wheels.
//
// Encoders exist ONLY on the rear-left and rear-right motors. The
// front motor on each side simply follows whatever the rear motor on
// that side is doing, with no independent feedback of its own. This
// is a real, permanent sensing limitation, not a bug - if the front
// and rear wheels on one side ever slip differently (likely on rough
// ground), the software has no way to know.
// ============================================================

// --- Motor driver pins (one L298N per side) ---
#define LEFT_PWM   5
#define LEFT_IN1   7
#define LEFT_IN2   8
#define RIGHT_PWM  6
#define RIGHT_IN1  9
#define RIGHT_IN2  10

// --- Encoder pins (rear motors only - the only ones with encoders) ---
// Pins 2 and 3 MUST stay here - they're the only two pins on the
// 328P with hardware external-interrupt support.
#define LEFT_ENC_A   2   // interrupt pin
#define LEFT_ENC_B   4
#define RIGHT_ENC_A  3   // interrupt pin
#define RIGHT_ENC_B  12

// --- Reserved for stepper/servo, not wired up by this firmware yet ---
#define SERVO_PIN 11
// MPU6050 IMU: SDA = A4, SCL = A5 (I2C, fixed pins, nothing to #define)

// --- Communication ---
// Must match the "baud_rate" hardware parameter in your ros2_control xacro.
#define BAUDRATE 115200

// --- Encoder scaling ---
// All 4 motors are the same model, so both sides SHOULD use one shared
// constant. MEASURE THIS YOURSELF before trusting it: send 'r' to zero
// the counters, spin a rear wheel exactly 10 full turns BY HAND, send
// 'e' to read the raw tick count, divide by 10. The previous project
// had left=730 vs right=655 on supposedly identical motors - a ~10%
// gap that's bigger than normal manufacturing tolerance and almost
// certainly a measurement slip, not a real hardware difference.
// If you genuinely confirm a repeatable left/right difference after
// measuring both sides carefully, split this into two constants and
// update RAD_S_TO_TICKS_S to use the correct one per side - don't
// guess, and don't assume the old numbers were right.
#define TICKS_PER_REV 730.0

// ROS sends target wheel velocity in rad/s. This converts rad/s ->
// encoder ticks/sec so the PID setpoint and the measured speed (also
// ticks/sec) are in the same units.
#define RAD_S_TO_TICKS_S(rad_s) ((rad_s) * TICKS_PER_REV / (2.0 * PI))

// --- Motor PWM ---
#define MOTOR_MAX_PWM 255
// Measured minimum PWM that actually turns the wheel (below this the
// motor draws current but doesn't move, which stalls the PID loop).
// Re-measure this on your actual hardware and update it.
#define MOTOR_MIN_PWM 20

// --- PID ---
#define PID_KP 1.5
#define PID_KI 0.1
#define PID_KD 0.01
// Error smaller than this (ticks/sec) is treated as "at target" and
// snapped to zero output, to stop standstill motor buzz from encoder
// jitter. Tune against your own encoder's noise floor.
#define PID_DEADZONE_TICKS_S 5.0

// How often the PID/motor/telemetry block runs, in milliseconds.
// NOTE: serial command parsing is NOT gated by this - it runs every
// single loop() iteration regardless, so incoming commands are never
// delayed behind this timer.
#define CONTROL_PERIOD_MS 33   // ~30Hz, matches a typical ros2_control update_rate

// --- Safety ---
// Hardware watchdog timeout - the chip itself resets if loop() ever
// hangs for longer than this (e.g. an I2C lockup on the IMU bus).
#define WDT_TIMEOUT_MS 2000
// Comms watchdog - stop the motors if no 'v' velocity command has
// arrived from the Pi within this long (dead teleop link / dead ROS).
#define COMMS_TIMEOUT_MS 500

#endif
