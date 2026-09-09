// serial_protocol.h
#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <Arduino.h>

class SerialProtocol {
public:
    bool parseVelocity(double &left, double &right);
    void sendTelemetry(long l_enc, long r_enc, float* acc, float* gyro);
};
#endif

