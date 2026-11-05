// I2C pin and bit-bang probe. Motors held off. No Wire library: the AVR's TWI hardware is never used.
// 1. Drive each line LOW and read it back (stays HIGH = short to VCC or a dead pin).
// 2. Software (bit-banged) I2C scan of every address, then WHO_AM_I at 0x68/0x69.
const uint8_t MOTOR_PINS[8] = {5, 6, 9, 10, 7, 8, A0, A1};
const uint8_t SDA_P = A3, SCL_P = A2;

void lineHigh(uint8_t p) { pinMode(p, INPUT_PULLUP); }          // release (open drain)
void lineLow(uint8_t p)  { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
void dly() { delayMicroseconds(20); }                             // ~20 kHz, slow and safe

void start() { lineHigh(SDA_P); lineHigh(SCL_P); dly(); lineLow(SDA_P); dly(); lineLow(SCL_P); dly(); }
void stop()  { lineLow(SDA_P); dly(); lineHigh(SCL_P); dly(); lineHigh(SDA_P); dly(); }
bool writeByte(uint8_t b) {           // returns true on ACK
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) lineHigh(SDA_P); else lineLow(SDA_P);
    dly(); lineHigh(SCL_P); dly(); lineLow(SCL_P);
  }
  lineHigh(SDA_P); dly(); lineHigh(SCL_P); dly();
  bool ack = digitalRead(SDA_P) == LOW;
  lineLow(SCL_P); dly();
  return ack;
}
uint8_t readByte(bool ack) {
  uint8_t b = 0; lineHigh(SDA_P);
  for (int i = 7; i >= 0; i--) {
    dly(); lineHigh(SCL_P); dly();
    if (digitalRead(SDA_P)) b |= (1 << i);
    lineLow(SCL_P);
  }
  if (ack) lineLow(SDA_P); else lineHigh(SDA_P);
  dly(); lineHigh(SCL_P); dly(); lineLow(SCL_P); lineHigh(SDA_P);
  return b;
}

void pinTest(uint8_t p, const char* name) {
  pinMode(p, INPUT); delay(2);
  int idle = digitalRead(p);
  lineLow(p); delayMicroseconds(50);
  int driven = digitalRead(p);
  lineHigh(p); delay(2);
  int released = digitalRead(p);
  Serial.print(name); Serial.print(F(": idle=")); Serial.print(idle);
  Serial.print(F(" driven_low_reads=")); Serial.print(driven);
  Serial.print(F(" released=")); Serial.print(released);
  Serial.println(driven ? F("  <-- CANNOT pull LOW (short to VCC or dead pin)") : F("  ok"));
}

void setup() {
  for (uint8_t i = 0; i < 8; i++) { pinMode(MOTOR_PINS[i], OUTPUT); digitalWrite(MOTOR_PINS[i], LOW); }
  Serial.begin(115200); delay(300);
  Serial.println(F("I2C PIN PROBE"));
  pinTest(SDA_P, "SDA(A3)");
  pinTest(SCL_P, "SCL(A2)");
  Serial.print(F("SOFT SCAN: "));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    start(); bool ack = writeByte(a << 1); stop();
    if (ack) { Serial.print(F("0x")); Serial.print(a, HEX); Serial.print(' '); found++; }
  }
  if (!found) Serial.print(F("nothing answered"));
  Serial.println();
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    start(); bool ok = writeByte(a << 1) && writeByte(0x75);
    start(); ok = ok && writeByte((a << 1) | 1);
    uint8_t who = ok ? readByte(false) : 0; stop();
    Serial.print(F("0x")); Serial.print(a, HEX);
    if (ok) { Serial.print(F(" WHO_AM_I=0x")); Serial.println(who, HEX); } else Serial.println(F(" no ACK"));
  }
  Serial.println(F("DONE"));
}
void loop() {}
