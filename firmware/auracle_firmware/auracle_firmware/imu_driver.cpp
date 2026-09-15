#include "imu_driver.h"

namespace {
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr float ACCEL_SCALE = 16384.0f;  // LSB per g, default +/-2g range
constexpr float GYRO_SCALE = 131.0f;     // LSB per deg/s, default +/-250dps range
constexpr float G_TO_MS2 = 9.80665f;
constexpr float DEG_TO_RAD = 0.017453293f;
}  // namespace

bool IMUDriver::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool IMUDriver::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, len);
    if (got != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

bool IMUDriver::begin() {
    Wire.begin();
    // Wake the chip - it boots in sleep mode (bit 6 of PWR_MGMT_1).
    // No WHO_AM_I check here on purpose: see imu_driver.h for why.
    return writeRegister(REG_PWR_MGMT_1, 0x00);
}

void IMUDriver::readData(float* acc, float* gyro) {
    uint8_t raw[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, raw, 14)) {
        // Leave acc/gyro as whatever the caller passed in on a dropped
        // read, rather than writing garbage - the .ino zero-inits
        // these each cycle, so a miss just looks like stale/no data.
        return;
    }

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];
    // raw[6..7] = temperature, unused here
    int16_t gx = (raw[8] << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    acc[0] = (ax / ACCEL_SCALE) * G_TO_MS2;
    acc[1] = (ay / ACCEL_SCALE) * G_TO_MS2;
    acc[2] = (az / ACCEL_SCALE) * G_TO_MS2;

    gyro[0] = (gx / GYRO_SCALE) * DEG_TO_RAD;
    gyro[1] = (gy / GYRO_SCALE) * DEG_TO_RAD;
    gyro[2] = (gz / GYRO_SCALE) * DEG_TO_RAD;
}