#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H
#include <Arduino.h>
#include <Wire.h>

// Raw I2C register driver - NOT the Adafruit_MPU6050 library.
// This board's MPU6050 clone reports WHO_AM_I = 0x72, not the 0x68 Adafruit's begin() hard-checks for, so that library always fails to init on this exact chip even though every actual register works fine (confirmed with a raw I2C read). This driver talks to the registers directly and skips that check entirely.
//
// Values are sent to the Pi as raw int16 counts (the Pi does the float
// scaling) - printing 6 floats per frame is slow on an AVR.
class IMUDriver {
public:
    bool begin();
    // Averages the gyro at rest and subtracts it from later reads.
    // Robot must be still - the Nano resets whenever the Pi opens the port.
    void calibrateGyro(uint16_t samples);
    // out = ax, ay, az, gx, gy, gz (raw counts, gyro bias removed)
    bool readRaw(int16_t* out);

private:
    static const uint8_t MPU_ADDR = 0x68;
    int16_t _gyroBias[3] = {0, 0, 0};
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
};
#endif
