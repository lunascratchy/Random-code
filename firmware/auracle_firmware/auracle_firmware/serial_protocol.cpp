#include "serial_protocol.h"
#include <stdlib.h>

void SerialProtocol::begin(long baud) {
    Serial.begin(baud);
}

void SerialProtocol::sendReady(bool imu_ok) {
    Serial.print(F("READY imu="));
    Serial.println(imu_ok ? 1 : 0);
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
            buf_len_ = 0;

            char *p = buf_;
            cmd.type = *p++;
            char *end;
            cmd.a1 = strtod(p, &end); p = end;
            cmd.a2 = strtod(p, &end); p = end;
            cmd.a3 = strtod(p, &end);
            return true;
        }

        if (buf_len_ < BUF_SIZE - 1) {
            buf_[buf_len_++] = c;
        } else {
            buf_len_ = 0;  // overlong garbage - drop it
        }
    }
    return false;
}

void SerialProtocol::sendTelemetry(unsigned long t_ms, long l_enc, long r_enc, const int16_t* imu) {
    // Integers only - AVR float printing is slow.
    Serial.print('e');
    Serial.print(' ');
    Serial.print(t_ms);
    Serial.print(' ');
    Serial.print(l_enc);
    Serial.print(' ');
    Serial.print(r_enc);
    if (imu) {
        Serial.print(F(" i"));
        for (uint8_t i = 0; i < 6; i++) {
            Serial.print(' ');
            Serial.print(imu[i]);
        }
    }
    Serial.print('\n');
}
