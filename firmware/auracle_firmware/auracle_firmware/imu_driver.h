#pragma once
#include <Arduino.h>

// Raw-register MPU6050 driver. This clone reports WHO_AM_I = 0x72, which
// the Adafruit library rejects, so the registers are driven directly.
class IMUDriver {
public:
    void begin();
    // Averages the gyro at rest; robot must be still.
    void calibrateGyro(uint16_t samples);
    // out = ax ay az gx gy gz, raw counts (gyro bias removed).
    // On a failed read, out keeps its previous values.
    void read(int16_t* out);

private:
    int16_t gyroBias_[3] = {0, 0, 0};
};
