// serial_protocol.cpp
#include "serial_protocol.h"
bool SerialProtocol::parseVelocity(double &left, double &right) {
    if (Serial.available() > 0) {
        char buf[32];
        int len = Serial.readBytesUntil('\n', buf, 31);
        buf[len] = '\0';
        if (buf[0] == 'v') {
            sscanf(buf, "v %lf %lf", &left, &right);
            return true;
        }
    }
    return false;
}
void SerialProtocol::sendTelemetry(long l_enc, long r_enc, float* acc, float* gyro) {
    Serial.print("e "); Serial.print(l_enc); Serial.print(" "); Serial.print(r_enc);
    Serial.print(" i ");
    for(int i=0; i<3; i++) { Serial.print(acc[i]); Serial.print(" "); }
    for(int i=0; i<3; i++) { Serial.print(gyro[i]); Serial.print(i==2 ? "" : " "); }
    Serial.println();
}