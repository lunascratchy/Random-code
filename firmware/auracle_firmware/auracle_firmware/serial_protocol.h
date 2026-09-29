#pragma once
#include <Arduino.h>

// ASCII lines, '\n' terminated.
// Pi -> Nano:
//   v <left_ticks_s> <right_ticks_s>   PID velocity target (rear wheels)
//   o <left_pwm> <right_pwm>           open-loop PWM (front-offset test)
//   s                                  stop
// Nano -> Pi:
//   READY                              once after boot
//   e <ms> <l_ticks> <r_ticks> <ax> <ay> <az> <gx> <gy> <gz>   every 50 ms
struct Command {
    char type = 0;
    float a = 0.0f, b = 0.0f;
};

class SerialProtocol {
public:
    void begin();
    void sendReady();
    // True once per complete line; call in a loop to drain.
    bool poll(Command& cmd);
    void sendTelemetry(unsigned long ms, long l, long r, const int16_t* imu);

private:
    static constexpr uint8_t BUF_SIZE = 32;
    char buf_[BUF_SIZE];
    uint8_t len_ = 0;
};
