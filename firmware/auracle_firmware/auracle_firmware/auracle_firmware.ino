#include <avr/wdt.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "serial_protocol.h"
#include "watchdog.h"


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
PIDController leftPID(PID_KP, PID_KI, PID_KD, SAMPLE_TIME_S);
PIDController rightPID(PID_KP, PID_KI, PID_KD, SAMPLE_TIME_S);
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
        Serial.println("WARN imu_init_failed");
    }

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), EncoderDriver::isrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), EncoderDriver::isrR, CHANGE);

    lastCommandTime = millis();
    lastControlTime = millis();
    protocol.sendReady();
}

void loop() {
    watchdog.pet();
    ParsedCommand cmd;
    if (protocol.update(cmd)) {
        switch (cmd.type) {
            case 'v':  // PID velocity target, rad/s
                lastCommandTime = millis();
                estopped = false;
                currentMode = MODE_PID;
                targetTicksL = RAD_S_TO_TICKS_S_L(cmd.a1);
                targetTicksR = RAD_S_TO_TICKS_S_R(cmd.a2);
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
                break;
        }
    }

    if (!watchdog.isCommsAlive(lastCommandTime)) {
        estopped = true;
        targetTicksL = 0;
        targetTicksR = 0;
    }

    unsigned long now = millis();
    unsigned long elapsedMs = now - lastControlTime;
    
    if (elapsedMs >= CONTROL_PERIOD_MS) {
        // FIX 1: Add the period to prevent schedule drift over time
        lastControlTime += CONTROL_PERIOD_MS;

        static long lastCountL = 0, lastCountR = 0;
        long countL = leftEnc.getCount();
        long countR = rightEnc.getCount();
        
        // FIX 2: Replace float division with a pre-calculated multiplier.
        // Because elapsedMs will almost always perfectly equal CONTROL_PERIOD_MS,
        // we multiply by the inverse (e.g., 1000/10 = 100).
        float inv_dt = 1000.0f / (float)elapsedMs; 
        float curTicksL = (float)(countL - lastCountL) * inv_dt;
        float curTicksR = (float)(countR - lastCountR) * inv_dt;
        
        lastCountL = countL;
        lastCountR = countR;

        if (estopped) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            leftPID.reset();
            rightPID.reset();
        } else if (currentMode == MODE_OPEN_LOOP) {
            // Cast double back to int for the motor driver
            leftMotor.setSpeed((int)openLoopL);
            rightMotor.setSpeed((int)openLoopR);
        } else {
            // FIX 3: Use the optimized PID compute (dt is handled internally now)
            leftMotor.setSpeed(leftPID.compute(targetTicksL, curTicksL));
            rightMotor.setSpeed(rightPID.compute(targetTicksR, curTicksR));
        }

        float acc[3] = {0, 0, 0};
        float gyro[3] = {0, 0, 0};
        if (imu_ok) {
            imu.readData(acc, gyro);
        }
        
        protocol.sendTelemetry(countL, countR, acc, gyro);
    }
}
