#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <Arduino.h>

// Commands understood, one line each, space-separated args:
//   v <left_rad_s> <right_rad_s>   - PID velocity target (normal driving)
//   o <left_pwm> <right_pwm>       - raw open-loop PWM passthrough (bench testing)
//   p <kp> <ki> <kd>               - live PID tuning
//   r                              - reset encoder counters
//   s                              - emergency stop
struct ParsedCommand {
    char type = 0;   // 0 = no command parsed this call
    double a1 = 0, a2 = 0, a3 = 0;
};

class SerialProtocol {
public:
    void begin(long baud);

    // Sends "READY\n" once from setup(), so the Pi's handshake
    // (ArduinoComms::waitForReady) has something to wait for.
    void sendReady();

    // Non-blocking: drains whatever bytes are currently available and
    // accumulates them ACROSS repeated calls into a fixed-size buffer -
    // never uses the Arduino String class, which does heap allocation
    // and can fragment the Nano's ~1.5KB free RAM to failure over many
    // hours of runtime. Call this every loop(). Returns true and fills
    // `cmd` only once a complete line has arrived.
    bool update(ParsedCommand &cmd);

    void sendTelemetry(long l_enc, long r_enc, float* acc, float* gyro);

private:
    static const uint8_t BUF_SIZE = 32;
    char buf_[BUF_SIZE];
    uint8_t buf_len_ = 0;
};
#endif
