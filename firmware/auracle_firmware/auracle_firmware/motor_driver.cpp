#include "motor_driver.h"
#include "config.h"

MotorDriver::MotorDriver(uint8_t pinFwd, uint8_t pinRev, uint8_t pinRearEn, uint8_t pinFrontEn,
                         uint8_t rearMin, uint8_t frontMin, float frontOffset)
    : pinFwd_(pinFwd), pinRev_(pinRev), pinRearEn_(pinRearEn), pinFrontEn_(pinFrontEn),
      rearMin_(rearMin), frontMin_(frontMin), frontOffset_(frontOffset) {}

void MotorDriver::begin() {
    pinMode(pinFwd_, OUTPUT);
    pinMode(pinRev_, OUTPUT);
    pinMode(pinRearEn_, OUTPUT);
    pinMode(pinFrontEn_, OUTPUT);
    stop();
}

void MotorDriver::stop() {
    analogWrite(pinRearEn_, 0);
    analogWrite(pinFrontEn_, 0);
    digitalWrite(pinFwd_, LOW);
    digitalWrite(pinRev_, LOW);
}

void MotorDriver::setEffort(int effort) {
    if (effort == 0) {
        stop();
        return;
    }
    bool forward = effort > 0;
    // Below the stall PWM the motor just hums - bump to the floor.
    int rear = constrain(abs(effort), rearMin_, MAX_PWM);
    int front = constrain((int)(rear * frontOffset_), frontMin_, MAX_PWM);

    digitalWrite(pinFwd_, forward ? HIGH : LOW);
    digitalWrite(pinRev_, forward ? LOW : HIGH);
    analogWrite(pinRearEn_, rear);
    analogWrite(pinFrontEn_, front);
}
