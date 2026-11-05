// Drive each candidate pin LOW / HIGH and read it back; also its pull-up level. Motor enables held LOW
// first, so the direction pins (7, 8, A0, A1) can be toggled without any motor moving.
// Encoder pins (2, 3, 4, 12) and serial (0, 1) are not driven.
const uint8_t EN[4] = {5, 6, 9, 10};
struct P { uint8_t pin; const char* name; };
const P PINS[] = {{A0,"A0"},{A1,"A1"},{A2,"A2"},{A3,"A3"},{A4,"A4"},{A5,"A5"},{7,"D7"},{8,"D8"},{11,"D11"},{13,"D13"}};
void setup() {
  for (uint8_t i = 0; i < 4; i++) { pinMode(EN[i], OUTPUT); digitalWrite(EN[i], LOW); }
  Serial.begin(115200); delay(300);
  Serial.println(F("PIN SCAN  (good pin: low=0 high=1 pullup=1)"));
  for (const P& p : PINS) {
    pinMode(p.pin, OUTPUT); digitalWrite(p.pin, LOW);  delayMicroseconds(100); int lo = digitalRead(p.pin);
    digitalWrite(p.pin, HIGH); delayMicroseconds(100); int hi = digitalRead(p.pin);
    pinMode(p.pin, INPUT_PULLUP); delay(2); int pu = digitalRead(p.pin);
    pinMode(p.pin, INPUT);
    Serial.print(p.name); Serial.print(F(": low=")); Serial.print(lo); Serial.print(F(" high=")); Serial.print(hi);
    Serial.print(F(" pullup=")); Serial.print(pu);
    Serial.println((lo == 0 && hi == 1 && pu == 1) ? F("  ok") : F("  <-- FAULTY"));
  }
  Serial.println(F("DONE"));
}
void loop() {}
