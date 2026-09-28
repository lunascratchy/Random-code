#include "imu_driver.h"

namespace {
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
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
    // 400kHz so a 14-byte read doesn't eat into the PID loop.
    Wire.setClock(400000);
    // Never let a flaky I2C bus hang loop() into a watchdog reset.
    Wire.setWireTimeout(3000, true);

    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) return false;  // wake
    delay(50);
    writeRegister(REG_CONFIG, 0x03);        // DLPF ~44Hz - cuts motor vibration noise
    writeRegister(REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s -> 131 LSB/(deg/s)
    writeRegister(REG_ACCEL_CONFIG, 0x00);  // +/-2g        -> 16384 LSB/g
    return true;
}

void IMUDriver::calibrateGyro(uint16_t samples) {
    int32_t sum[3] = {0, 0, 0};
    uint16_t good = 0;
    uint8_t raw[6];
    for (uint16_t i = 0; i < samples; i++) {
        if (readRegisters(0x43, raw, 6)) {  // GYRO_XOUT_H
            for (uint8_t k = 0; k < 3; k++) {
                sum[k] += (int16_t)((uint16_t)raw[2 * k] << 8 | raw[2 * k + 1]);
            }
            good++;
        }
        delay(2);
    }
    if (good == 0) return;
    for (uint8_t k = 0; k < 3; k++) {
        _gyroBias[k] = (int16_t)(sum[k] / good);
    }
}

bool IMUDriver::readRaw(int16_t* out) {
    uint8_t raw[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, raw, 14)) {
        return false;
    }
    // raw[6..7] is temperature - skipped.
    out[0] = (int16_t)((uint16_t)raw[0] << 8 | raw[1]);
    out[1] = (int16_t)((uint16_t)raw[2] << 8 | raw[3]);
    out[2] = (int16_t)((uint16_t)raw[4] << 8 | raw[5]);
    out[3] = (int16_t)((uint16_t)raw[8] << 8 | raw[9]) - _gyroBias[0];
    out[4] = (int16_t)((uint16_t)raw[10] << 8 | raw[11]) - _gyroBias[1];
    out[5] = (int16_t)((uint16_t)raw[12] << 8 | raw[13]) - _gyroBias[2];
    return true;
}
