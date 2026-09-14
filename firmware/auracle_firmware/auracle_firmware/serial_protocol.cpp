#include "serial_protocol.h"
#include <stdlib.h>  // strtod

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
            continue;  // ignore - wait for the '\n' that follows
        }

        if (c == '\n') {
            if (buf_len_ == 0) {
                continue;  // blank line, nothing to parse
            }
            buf_[buf_len_] = '\0';

            // Manual token parsing with strtod(), NOT sscanf("%f"/"%lf").
            // The default avr-libc sscanf linked by the Arduino IDE does
            // NOT include floating-point conversion support - "%f"/"%lf"
            // silently parses as 0.0 instead of failing loudly, which
            // would make every velocity command look like "stop" with
            // no error anywhere. strtod() is a separate function that
            // avr-libc DOES implement fully, with no extra linker flags
            // needed.
            char *p = buf_;
            cmd.type = *p;
            p++;

            char *end;
            while (*p == ' ') p++;
            cmd.a1 = strtod(p, &end);
            p = end;
            while (*p == ' ') p++;
            cmd.a2 = strtod(p, &end);
            p = end;
            while (*p == ' ') p++;
            cmd.a3 = strtod(p, &end);

            buf_len_ = 0;
            return true;
        }

        if (buf_len_ < BUF_SIZE - 1) {
            buf_[buf_len_++] = c;
        } else {
            // Line too long / noise on the line with no terminator -
            // drop it and resync on the next '\n' rather than growing
            // forever or misparsing garbage.
            buf_len_ = 0;
        }
    }
    return false;
}

void SerialProtocol::sendTelemetry(long l_enc, long r_enc, float* acc, float* gyro) {
    Serial.print("e ");
    Serial.print(l_enc);
    Serial.print(" ");
    Serial.print(r_enc);
    Serial.print(" i ");
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
