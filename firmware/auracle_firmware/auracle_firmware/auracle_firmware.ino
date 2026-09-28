#include <avr/wdt.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "serial_protocol.h"
#include "watchdog.h"

// Clear the watchdog flag as early as possible after a WDT reset, or
// older bootloaders get stuck in a reset loop.
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void get_mcusr(void) __attribute__((naked, used, section(".init3")));
void get_mcusr(void) {
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

enum ControlMode { MODE_STOPPED, MODE_PID, MODE_OPEN_LOOP };

MotorDriver leftSide(PIN_L_FWD, PIN_L_REV, PIN_LB_EN, PIN_LF_EN, LB_MIN_PWM, LF_MIN_PWM, LF_OFFSET);
MotorDriver rightSide(PIN_R_FWD, PIN_R_REV, PIN_RB_EN, PIN_RF_EN, RB_MIN_PWM, RF_MIN_PWM, RF_OFFSET);
EncoderDriver leftEnc(PIN_LB_ENC_INT, PIN_LB_ENC_DIR);
EncoderDriver rightEnc(PIN_RB_ENC_INT, PIN_RB_ENC_DIR);
PIDController leftPID(PID_KP, PID_KI, PID_KD, PID_DEADZONE_TICKS_S);
PIDController rightPID(PID_KP, PID_KI, PID_KD, PID_DEADZONE_TICKS_S);
IMUDriver imu;
SerialProtocol protocol;
Watchdog watchdog;

bool imu_ok = false;
ControlMode mode = MODE_STOPPED;

unsigned long lastCommandTime = 0;   // comms watchdog
unsigned long lastControlTime = 0;   // control loop scheduler (ms)
unsigned long lastSampleUs = 0;      // exact dt for speed measurement
long lastCountL = 0, lastCountR = 0;

float targetL = 0, targetR = 0;      // ticks/sec
int openLoopL = 0, openLoopR = 0;    // signed PWM

float clampTarget(float t) {
    if (fabs(t) < MIN_TARGET_TICKS_S) return 0.0f;
    return constrain(t, -MAX_TICKS_S, MAX_TICKS_S);
}

void stopAll() {
    mode = MODE_STOPPED;
    targetL = targetR = 0;
    leftPID.reset();
    rightPID.reset();
    leftSide.stop();
    rightSide.stop();
}

void handleCommand(const ParsedCommand &cmd) {
    switch (cmd.type) {
        case 'v':
            lastCommandTime = millis();
            targetL = clampTarget(cmd.a1);
            targetR = clampTarget(cmd.a2);
            if (mode != MODE_PID) {
                leftPID.reset();
                rightPID.reset();
                mode = MODE_PID;
            }
            break;

        case 'o':
            lastCommandTime = millis();
            openLoopL = constrain((int)cmd.a1, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
            openLoopR = constrain((int)cmd.a2, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
            mode = MODE_OPEN_LOOP;
            break;

        case 'p':
            leftPID.setTunings(cmd.a1, cmd.a2, cmd.a3);
            rightPID.setTunings(cmd.a1, cmd.a2, cmd.a3);
            break;

        case 'r':
            leftEnc.reset();
            rightEnc.reset();
            lastCountL = lastCountR = 0;
            break;

        case 's':
            stopAll();
            break;

        default:
            break;
    }
}

void setup() {
    leftSide.begin();
    rightSide.begin();
    stopAll();

    protocol.begin(BAUDRATE);

    leftEnc.begin();
    rightEnc.begin();
    EncoderDriver::instanceL = &leftEnc;
    EncoderDriver::instanceR = &rightEnc;
    attachInterrupt(digitalPinToInterrupt(PIN_LB_ENC_INT), EncoderDriver::isrL, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RB_ENC_INT), EncoderDriver::isrR, RISING);

    imu_ok = imu.begin();
    if (imu_ok) {
        imu.calibrateGyro(200);  // ~0.5 s, robot must be still
    }

    // Started after gyro calibration so that doesn't trip it.
    watchdog.begin();

    lastCommandTime = millis();
    lastControlTime = millis();
    lastSampleUs = micros();
    protocol.sendReady(imu_ok);
}

void loop() {
    watchdog.pet();

    ParsedCommand cmd;
    while (protocol.update(cmd)) {
        handleCommand(cmd);
    }

    if (mode != MODE_STOPPED && !watchdog.isCommsAlive(lastCommandTime)) {
        stopAll();
    }

    unsigned long now = millis();
    if (now - lastControlTime < CONTROL_PERIOD_MS) {
        return;
    }
    lastControlTime += CONTROL_PERIOD_MS;
    if (now - lastControlTime > CONTROL_PERIOD_MS) {
        lastControlTime = now;  // fell behind (e.g. I2C stall) - don't burst to catch up
    }

    unsigned long nowUs = micros();
    float dt = (nowUs - lastSampleUs) * 1e-6f;
    lastSampleUs = nowUs;

    long countL = leftEnc.getCount();
    long countR = rightEnc.getCount();
    float speedL = (countL - lastCountL) / dt;
    float speedR = (countR - lastCountR) / dt;
    lastCountL = countL;
    lastCountR = countR;

    if (mode == MODE_PID) {
        leftSide.setEffort(leftPID.compute(targetL, speedL, dt));
        rightSide.setEffort(rightPID.compute(targetR, speedR, dt));
    } else if (mode == MODE_OPEN_LOOP) {
        leftSide.setEffort(openLoopL);
        rightSide.setEffort(openLoopR);
    }

    int16_t imuRaw[6];
    bool haveImu = imu_ok && imu.readRaw(imuRaw);
    protocol.sendTelemetry(now, countL, countR, haveImu ? imuRaw : nullptr);
}
