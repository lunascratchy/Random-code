#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <Arduino.h>

struct ParsedCommand {
    char type = 0;  
    float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f; // Changed to float
};

class SerialProtocol {
public:
    void begin(long baud);
    void sendReady();
    bool update(ParsedCommand &cmd);

    void sendTelemetry(long l_enc, long r_enc, float* acc, float* gyro);

private:
    static const uint8_t BUF_SIZE = 64; // Increased to 64 to prevent truncation
    char buf_[BUF_SIZE];
    uint8_t buf_len_ = 0;
};
#endif