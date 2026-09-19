#include "serial_protocol.h"
#include <stdlib.h>  

void SerialProtocol::begin(long baud) {
    Serial.begin(baud);
}

void SerialProtocol::sendReady() {
    Serial.println("READY");
}

bool SerialProtocol::update(ParsedCommand &cmd) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r') {
            continue; 
        }

        if (c == '\n') {
            if (buf_len_ == 0) {
                continue;  
            }
            buf_[buf_len_] = '\0';
            char *p = buf_;
            cmd.type = *p;
            p++;

            char *end;
            while (*p == ' ') p++;
            cmd.a1 = strtof(p, &end);
            p = end;
            while (*p == ' ') p++;
            cmd.a2 = strtof(p, &end);
            p = end;
            while (*p == ' ') p++;
            cmd.a3 = strtof(p, &end);

            buf_len_ = 0;
            return true;
        }

        if (buf_len_ < BUF_SIZE - 1) {
            buf_[buf_len_++] = c;
        } else {
            buf_len_ = 0; 
        }
    }
    return false;
}

void SerialProtocol::sendTelemetry(long l_enc, long r_enc, float* acc, float* gyro) {
    if (Serial.availableForWrite() < 50) {
        return; 
    }

    Serial.print("e ");
    Serial.print(l_enc);
    Serial.print(" ");
    Serial.print(r_enc);
    Serial.print(" i ");
    
    // Arduino print(float) defaults to 2 decimal places. 
    // Kept as default for speed, add ,4 inside print() if you need more precision.
    for (int i = 0; i < 3; i++) {
        Serial.print(acc[i]);
        Serial.print(" ");
    }
    for (int i = 0; i < 3; i++) {
        Serial.print(gyro[i]);
        Serial.print(i == 2 ? "" : " ");
    }
    Serial.println();
}