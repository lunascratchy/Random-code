#include <avr/wdt.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "serial_protocol.h"
#include "watchdog.h"

// Runs before global constructors / main(), as early as possible after
// reset. Some Nano bootloaders don't clear the "watchdog caused this
// reset" flag, so if the WDT is left enabled across a watchdog-triggered
// reset, the chip can boot-loop forever before setup() ever gets a
// chance to re-arm it deliberately. Clearing MCUSR and disabling the WDT
// here breaks that loop; watchdog.begin() in setup() re-enables it with
// the real timeout.
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void get_mcusr(void) __attribute__((naked, used, section(".init3")));
void get_mcusr(void) {
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

enum ControlMode { MODE_PID, MODE_OPEN_LOOP };

MotorDriver leftMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2);
MotorDriver rightMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2);
EncoderDriver leftEnc(LEFT_ENC_A, LEFT_ENC_B);
EncoderDriver rightEnc(RIGHT_ENC_A, RIGHT_ENC_B);
PIDController leftPID(PID_KP, PID_KI, PID_KD);
PIDController rightPID(PID_KP, PID_KI, PID_KD);
IMUDriver imu;
SerialProtocol protocol;
Watchdog watchdog;

// Link ISRs
EncoderDriver* EncoderDriver::instanceL = &leftEnc;
EncoderDriver* EncoderDriver::instanceR = &rightEnc;

bool imu_ok = false;
ControlMode currentMode = MODE_PID;
bool estopped = false;

unsigned long lastCommandTime = 0;   // resets the comms watchdog
unsigned long lastControlTime = 0;   // gates the PID/motor/telemetry block

double targetTicksL = 0, targetTicksR = 0;
double openLoopL = 0, openLoopR = 0;

void setup() {
    protocol.begin(BAUDRATE);
    watchdog.begin();

    imu_ok = imu.begin();
    if (!imu_ok) {
        // Don't halt - the robot can still drive without IMU data, the
        // hardware interface will just receive stale/zero IMU state.
        // Make it visible on the serial monitor for debugging.
        Serial.println("WARN imu_init_failed");
    }

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), EncoderDriver::isrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), EncoderDriver::isrR, CHANGE);

    lastCommandTime = millis();
    lastControlTime = millis();

    // Lets the Pi's ArduinoComms::waitForReady() know we're actually up,
    // rather than racing the Nano's post-upload auto-reset and reading
    // garbage/nothing for the first ~1-2s.
    protocol.sendReady();
}

void loop() {
    // 1. Pet the hardware watchdog immediately, every loop, before
    //    anything else that could conceivably stall.
    watchdog.pet();

    // 2. Parse serial every loop, UNGATED - never delayed behind the
    //    control-period timer below, so commands are always picked up
    //    as soon as they arrive.
    ParsedCommand cmd;
    if (protocol.update(cmd)) {
        switch (cmd.type) {
            case 'v':  // PID velocity target, rad/s
                lastCommandTime = millis();
                estopped = false;
                currentMode = MODE_PID;
                targetTicksL = RAD_S_TO_TICKS_S(cmd.a1);
                targetTicksR = RAD_S_TO_TICKS_S(cmd.a2);
                break;

            case 'o':  // raw open-loop PWM, bench testing only
                lastCommandTime = millis();
                estopped = false;
                currentMode = MODE_OPEN_LOOP;
                openLoopL = cmd.a1;
                openLoopR = cmd.a2;
                break;

            case 'p':  // live PID tuning
                leftPID.setTunings(cmd.a1, cmd.a2, cmd.a3);
                rightPID.setTunings(cmd.a1, cmd.a2, cmd.a3);
                break;

            case 'r':  // zero the encoder counters
                leftEnc.reset();
                rightEnc.reset();
                break;

            case 's':  // emergency stop
                estopped = true;
                currentMode = MODE_PID;
                targetTicksL = 0;
                targetTicksR = 0;
                leftPID.reset();
                rightPID.reset();
                break;

            default:
                // Unknown/garbage command byte - ignore rather than
                // acting on it or polluting anything downstream.
                break;
        }
    }

    // 3. Comms safety check: if the Pi goes quiet, stop regardless of
    //    what mode we were in.
    if (!watchdog.isCommsAlive(lastCommandTime)) {
        estopped = true;
        targetTicksL = 0;
        targetTicksR = 0;
    }

    // 4. Fixed-period control block: encoder deltas, PID, motor output,
    //    telemetry. Gated by ELAPSED TIME, not delay() - so step 2's
    //    serial parsing above is never blocked waiting for this.
    unsigned long now = millis();
    unsigned long elapsedMs = now - lastControlTime;
    if (elapsedMs >= CONTROL_PERIOD_MS) {
        double dt = elapsedMs / 1000.0;
        lastControlTime = now;

        static long lastCountL = 0, lastCountR = 0;
        long countL = leftEnc.getCount();
        long countR = rightEnc.getCount();
        double curTicksL = (countL - lastCountL) / dt;
        double curTicksR = (countR - lastCountR) / dt;
        lastCountL = countL;
        lastCountR = countR;

        if (estopped) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            leftPID.reset();
            rightPID.reset();
        } else if (currentMode == MODE_OPEN_LOOP) {
            // Direct PWM passthrough - PID is NOT allowed to run here,
            // or it would immediately overwrite this every cycle.
            leftMotor.setSpeed(openLoopL);
            rightMotor.setSpeed(openLoopR);
        } else {
            leftMotor.setSpeed(leftPID.compute(targetTicksL, curTicksL, dt));
            rightMotor.setSpeed(rightPID.compute(targetTicksR, curTicksR, dt));
        }

        float acc[3] = {0, 0, 0};
        float gyro[3] = {0, 0, 0};
        if (imu_ok) {
            imu.readData(acc, gyro);
        }
        protocol.sendTelemetry(countL, countR, acc, gyro);
    }
}
