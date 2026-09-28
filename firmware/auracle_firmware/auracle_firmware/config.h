#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// ============================================================
// PINOUT - matches the bench-tested wiring (README "Real robot").
// Direction is bridged per side (IN1+IN3 / IN2+IN4), speed is
// independent per wheel (ENA/ENB jumper caps removed).
// ============================================================
// --- Right L298N ---
#define PIN_RF_EN   5    // ENA - Right Front PWM (Timer0)
#define PIN_RB_EN   6    // ENB - Right Back PWM  (Timer0)
#define PIN_R_FWD   7    // IN1 + IN3 bridged
#define PIN_R_REV   8    // IN2 + IN4 bridged
// --- Left L298N ---
#define PIN_LB_EN   9    // ENA - Left Back PWM   (Timer1)
#define PIN_LF_EN   10   // ENB - Left Front PWM  (Timer1)
#define PIN_L_FWD   A0   // IN1 + IN3 bridged
#define PIN_L_REV   A1   // IN2 + IN4 bridged
// Do NOT use the Servo library: it takes over Timer1 and kills PWM on D9/D10.

// --- Encoders (rear motors only) ---
// 1x decoding, same as the bench tests: interrupt on RISING of one
// channel, read the other for direction (HIGH = forward = count up).
#define PIN_LB_ENC_INT  2    // INT0 - LB encoder A (C1)
#define PIN_LB_ENC_DIR  4    //        LB encoder B (C2)
#define PIN_RB_ENC_INT  3    // INT1 - RB encoder B
#define PIN_RB_ENC_DIR  12   //        RB encoder A

// MPU6050 IMU: SDA = A4, SCL = A5 (fixed I2C pins). Optional - if it
// doesn't answer, telemetry just omits the IMU fields.

// --- Serial ---
// Must match "baud_rate" in ros2_control.xacro.
#define BAUDRATE 115200

// ============================================================
// MOTOR CHARACTERISATION (from the stall/min-PWM ramp test)
// ============================================================
#define LB_MIN_PWM 45
#define RB_MIN_PWM 60
#define LF_MIN_PWM 90
#define RF_MIN_PWM 90
#define MOTOR_MAX_PWM 255

// Front wheels have no encoders: PWM = rear commanded PWM * offset.
// 1.0 = no correction. Tune once you've done the front-offset test.
#define LF_OFFSET 1.0f
#define RF_OFFSET 1.0f

// ============================================================
// PID (rear wheels, incremental form - same as the tuned teleop sketch)
// effort += Kp*e + Ki*integral + Kd*derivative, units = ticks/sec.
// ============================================================
#define PID_KP 0.08f
#define PID_KI 0.0f
#define PID_KD 0.0f

// Control loop period. Kp was tuned at 50 ms - changing this changes
// the effective gain (effort is accumulated once per period).
#define CONTROL_PERIOD_MS 50

// Speed resolution at 1x decoding = 1 tick / CONTROL_PERIOD = 20 ticks/s.
// A steady wheel reads N or N+1 ticks per window, so the error dithers
// by +/-20 ticks/s even when perfectly on target. Deadzone just above
// one quantum stops the PID chasing that jitter. If the error trace
// still hops by ~40 at steady state, raise to ~45.
#define PID_DEADZONE_TICKS_S 25.0f

// Targets below this are treated as "stop": the min-PWM floors would
// otherwise make the wheel jump to 45-90 PWM for a 3 ticks/s request.
#define MIN_TARGET_TICKS_S 25.0f

// Highest speed validated on the bench. Targets are clamped to this.
#define MAX_TICKS_S 1400.0f

// ============================================================
// SAFETY
// ============================================================
#define WDT_TIMEOUT_MS   2000   // hardware reset if loop() hangs
#define COMMS_TIMEOUT_MS 500    // motors stop if the Pi goes quiet

#endif
