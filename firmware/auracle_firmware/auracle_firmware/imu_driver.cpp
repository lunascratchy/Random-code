#include "imu_driver.h"
#include <Wire.h>

static constexpr uint8_t ADDR = 0x68;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;

static void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDR, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static int16_t be16(const uint8_t* p) { return (int16_t)((uint16_t)p[0] << 8 | p[1]); }

void IMUDriver::begin() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);  // a flaky bus must never hang loop()
    writeReg(REG_PWR_MGMT_1, 0x00);   // wake
    delay(50);
    writeReg(REG_CONFIG, 0x03);        // DLPF ~44 Hz
    writeReg(REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s
    writeReg(REG_ACCEL_CONFIG, 0x00);  // +/-2 g
}

void IMUDriver::calibrateGyro(uint16_t samples) {
    int32_t sum[3] = {0, 0, 0};
    uint16_t good = 0;
    uint8_t raw[6];
    for (uint16_t i = 0; i < samples; i++) {
        if (readRegs(REG_GYRO_XOUT_H, raw, 6)) {
            for (uint8_t k = 0; k < 3; k++) sum[k] += be16(raw + 2 * k);
            good++;
        }
        delay(2);
    }
    if (good == 0) return;
    for (uint8_t k = 0; k < 3; k++) gyroBias_[k] = (int16_t)(sum[k] / good);
}

void IMUDriver::read(int16_t* out) {
    uint8_t raw[14];
    if (!readRegs(REG_ACCEL_XOUT_H, raw, 14)) return;
    // raw[6..7] is temperature - skipped.
    out[0] = be16(raw + 0);
    out[1] = be16(raw + 2);
    out[2] = be16(raw + 4);
    out[3] = be16(raw + 8) - gyroBias_[0];
    out[4] = be16(raw + 10) - gyroBias_[1];
    out[5] = be16(raw + 12) - gyroBias_[2];
}
