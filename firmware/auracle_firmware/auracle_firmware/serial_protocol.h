#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <Arduino.h>

// Line protocol (ASCII, '\n' terminated). Pi -> Nano:
//   v <left_ticks_s> <right_ticks_s>   PID velocity target (rear encoder ticks/sec)
//   o <left_pwm> <right_pwm>           open-loop signed rear PWM (bench testing)
//   p <kp> <ki> <kd>                   live PID retune (not saved)
//   r                                  zero encoder counts
//   s                                  stop
// Nano -> Pi:
//   READY imu=<0|1>                    once after boot
//   e <millis> <l_ticks> <r_ticks>[ i ax ay az gx gy gz]   every control period
//
// ticks/sec (not rad/s) goes over the wire so ticks-per-rev lives in one
// place: the ros2_control xacro.
struct ParsedCommand {
    char type = 0;
    float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f;
};

class SerialProtocol {
public:
    void begin(long baud);
    void sendReady(bool imu_ok);
    // Returns true once per complete line; call in a loop to drain.
    bool update(ParsedCommand &cmd);
    // imu may be nullptr when the IMU is absent.
    void sendTelemetry(unsigned long t_ms, long l_enc, long r_enc, const int16_t* imu);

private:
    static const uint8_t BUF_SIZE = 48;
    char buf_[BUF_SIZE];
    uint8_t buf_len_ = 0;
};
#endif
