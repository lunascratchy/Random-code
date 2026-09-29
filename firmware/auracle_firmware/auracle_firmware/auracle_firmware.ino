#include <avr/wdt.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "serial_protocol.h"

// Clear the WDT flag right after reset, or older bootloaders reset-loop.
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void get_mcusr(void) __attribute__((naked, used, section(".init3")));
void get_mcusr(void) {
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

enum Mode : uint8_t { MODE_STOPPED, MODE_PID, MODE_OPEN_LOOP };

MotorDriver leftSide(PIN_L_FWD, PIN_L_REV, PIN_LB_EN, PIN_LF_EN, LB_MIN_PWM, LF_MIN_PWM, LF_OFFSET);
MotorDriver rightSide(PIN_R_FWD, PIN_R_REV, PIN_RB_EN, PIN_RF_EN, RB_MIN_PWM, RF_MIN_PWM, RF_OFFSET);
EncoderDriver leftEnc(PIN_LB_ENC_INT, PIN_LB_ENC_DIR);
EncoderDriver rightEnc(PIN_RB_ENC_INT, PIN_RB_ENC_DIR);
PIDController leftPid, rightPid;
IMUDriver imu;
SerialProtocol link;

Mode mode = MODE_STOPPED;
float targetL = 0, targetR = 0;  // ticks/s
int pwmL = 0, pwmR = 0;          // open loop
unsigned long lastCmdMs = 0, lastCtrlMs = 0, lastSampleUs = 0;
long lastCountL = 0, lastCountR = 0;
int16_t imuRaw[6] = {0, 0, 0, 0, 0, 0};

void isrLeft() { leftEnc.tick(); }
void isrRight() { rightEnc.tick(); }

float clampTarget(float t) {
    if (fabs(t) < MIN_TARGET_TICKS_S) return 0.0f;
    return constrain(t, -MAX_TICKS_S, MAX_TICKS_S);
}

void stopAll() {
    mode = MODE_STOPPED;
    targetL = targetR = 0;
    leftPid.reset();
    rightPid.reset();
    leftSide.stop();
    rightSide.stop();
}

void handle(const Command& cmd) {
    switch (cmd.type) {
        case 'v':
            lastCmdMs = millis();
            targetL = clampTarget(cmd.a);
            targetR = clampTarget(cmd.b);
            if (mode != MODE_PID) {
                leftPid.reset();
                rightPid.reset();
                mode = MODE_PID;
            }
            break;
        case 'o':
            lastCmdMs = millis();
            pwmL = constrain((int)cmd.a, -MAX_PWM, MAX_PWM);
            pwmR = constrain((int)cmd.b, -MAX_PWM, MAX_PWM);
            mode = MODE_OPEN_LOOP;
            break;
        case 's':
            stopAll();
            break;
    }
}

void setup() {
    leftSide.begin();
    rightSide.begin();
    link.begin();

    leftEnc.begin();
    rightEnc.begin();
    attachInterrupt(digitalPinToInterrupt(PIN_LB_ENC_INT), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RB_ENC_INT), isrRight, RISING);

    imu.begin();
    imu.calibrateGyro(200);  // ~0.5 s, robot must be still

    wdt_enable(WDTO_2S);  // after calibration so it can't trip it
    lastCmdMs = lastCtrlMs = millis();
    lastSampleUs = micros();
    link.sendReady();
}

void loop() {
    wdt_reset();

    Command cmd;
    while (link.poll(cmd)) handle(cmd);

    unsigned long now = millis();
    if (mode != MODE_STOPPED && now - lastCmdMs > COMMS_TIMEOUT_MS) stopAll();

    if (now - lastCtrlMs < CONTROL_PERIOD_MS) return;
    lastCtrlMs += CONTROL_PERIOD_MS;
    if (now - lastCtrlMs > CONTROL_PERIOD_MS) lastCtrlMs = now;  // fell behind - don't burst

    unsigned long nowUs = micros();
    float dt = (nowUs - lastSampleUs) * 1e-6f;
    lastSampleUs = nowUs;

    long countL = leftEnc.count();
    long countR = rightEnc.count();
    float speedL = (countL - lastCountL) / dt;
    float speedR = (countR - lastCountR) / dt;
    lastCountL = countL;
    lastCountR = countR;

    if (mode == MODE_PID) {
        leftSide.setEffort(leftPid.compute(targetL, speedL, dt));
        rightSide.setEffort(rightPid.compute(targetR, speedR, dt));
    } else if (mode == MODE_OPEN_LOOP) {
        leftSide.setEffort(pwmL);
        rightSide.setEffort(pwmR);
    }

    imu.read(imuRaw);
    link.sendTelemetry(now, countL, countR, imuRaw);
}
