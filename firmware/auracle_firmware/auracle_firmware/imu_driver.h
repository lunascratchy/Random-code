#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H
#include <Wire.h>

// Raw I2C register driver - NOT the Adafruit_MPU6050 library.
// This board's MPU6050 clone reports WHO_AM_I = 0x72, not the 0x68
// Adafruit's begin() hard-checks for, so that library always fails to
// init on this exact chip even though every actual register works
// fine (confirmed with a raw I2C read). This driver talks to the
// registers directly and skips that check entirely.
class IMUDriver {
public:
    bool begin();
    void readData(float* acc, float* gyro);

private:
    static const uint8_t MPU_ADDR = 0x68;
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
};
#endif