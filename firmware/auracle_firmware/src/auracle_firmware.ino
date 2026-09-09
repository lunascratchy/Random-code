#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "serial_protocol.h"
#include "watchdog.h"

// Instantiate objects
MotorDriver leftMotor(L_MOTOR_PWM, L_MOTOR_IN1, L_MOTOR_IN2);
MotorDriver rightMotor(R_MOTOR_PWM, R_MOTOR_IN1, R_MOTOR_IN2);
EncoderDriver leftEnc(L_ENC_A, L_ENC_B);
EncoderDriver rightEnc(R_ENC_A, R_ENC_B);
PIDController leftPID(PID_KP, PID_KI, PID_KD);
PIDController rightPID(PID_KP, PID_KI, PID_KD);
IMUDriver imu;
SerialProtocol protocol;
Watchdog watchdog;

unsigned long lastRosPacketTime = 0;

// Link ISRs
EncoderDriver* EncoderDriver::instanceL = &leftEnc;
EncoderDriver* EncoderDriver::instanceR = &rightEnc;

void setup() {
    Serial.begin(BAUDRATE);
    watchdog.begin();
    imu.begin();
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), EncoderDriver::isrL, RISING);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), EncoderDriver::isrR, RISING);
}

void loop() {
    // 1. Pet the hardware watchdog immediately
    watchdog.pet();

    double targetL = 0, targetR = 0;
    if (protocol.parseVelocity(targetL, targetR)) {
        lastRosPacketTime = millis(); // Update the timestamp when we hear from ROS
    }

    // 2. Communication Safety Check
    if (!watchdog.isCommsAlive(lastRosPacketTime)) {
        // EMERGENCY STOP: ROS is gone, stop the motors!
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        targetL = 0;
        targetR = 0;
    }

    // PID loop to control motor PWM
    double curL = (leftEnc.getCount() / 100.0); // simplified speed
    double curR = (rightEnc.getCount() / 100.0);
    
    leftMotor.setSpeed(leftPID.compute(targetL, curL, dt));
    rightMotor.setSpeed(rightPID.compute(targetR, curR, dt));

    // Telemetry
    float acc[3], gyro[3];
    imu.readData(acc, gyro);
    protocol.sendTelemetry(leftEnc.getCount(), rightEnc.getCount(), acc, gyro);
    
    delay(33); // ~30Hz loop
}