// IMU probe for the Auracle Nano. Motors are held off.
// 1. Checks the I2C wires at rest (SDA = A4, SCL = A5 should both sit HIGH: the module's pull-ups).
// 2. Scans every I2C address and lists what answers.
// 3. For 0x68 / 0x69 (MPU6050 with AD0 low / high): reads WHO_AM_I, wakes it, prints raw accel/gyro.
#include <Wire.h>

const uint8_t MOTOR_PINS[8] = {5, 6, 9, 10, 7, 8, A0, A1};

uint8_t readReg(uint8_t addr, uint8_t reg, bool &ok) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { ok = false; return 0; }
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) { ok = false; return 0; }
  ok = true;
  return Wire.read();
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}

void probe() {
  pinMode(A4, INPUT); pinMode(A5, INPUT);   // no internal pull-ups: see what the module does
  delay(5);
  Serial.print(F("LINES  SDA(A4)=")); Serial.print(digitalRead(A4));
  Serial.print(F("  SCL(A5)=")); Serial.println(digitalRead(A5));

  // Bus recovery: a device stuck mid-transfer holds SDA low. Clock SCL 9 times until it lets go.
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  uint8_t pulses = 0;
  for (; pulses < 9 && digitalRead(A4) == LOW; pulses++) {
    digitalWrite(A5, LOW); delayMicroseconds(10); digitalWrite(A5, HIGH); delayMicroseconds(10);
  }
  pinMode(A5, INPUT_PULLUP);
  delay(2);
  Serial.print(F("RECOVERY  SDA low pulses needed=")); Serial.print(pulses);
  Serial.print(F("  now SDA=")); Serial.print(digitalRead(A4));
  Serial.print(F(" SCL=")); Serial.println(digitalRead(A5));

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  Serial.print(F("SCAN  "));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    uint8_t e = Wire.endTransmission();
    if (e == 0) { Serial.print(F("0x")); Serial.print(a, HEX); Serial.print(' '); found++; }
    if (Wire.getWireTimeoutFlag()) { Serial.print(F("(bus TIMEOUT) ")); Wire.clearWireTimeoutFlag(); break; }
  }
  if (!found) Serial.print(F("nothing answered"));
  Serial.println();

  for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
    bool ok;
    uint8_t who = readReg(addr, 0x75, ok);
    Serial.print(F("ADDR 0x")); Serial.print(addr, HEX);
    if (!ok) { Serial.println(F("  no answer")); continue; }
    Serial.print(F("  WHO_AM_I=0x")); Serial.print(who, HEX);
    Serial.print(F("  PWR_MGMT_1=0x")); Serial.println(readReg(addr, 0x6B, ok), HEX);
    writeReg(addr, 0x6B, 0x00);   // wake
    delay(50);
    for (uint8_t n = 0; n < 3; n++) {
      Wire.beginTransmission(addr); Wire.write(0x3B); Wire.endTransmission(false);
      if (Wire.requestFrom(addr, (uint8_t)14) != 14) { Serial.println(F("  RAW read failed")); break; }
      int16_t v[7];
      for (uint8_t i = 0; i < 7; i++) v[i] = (int16_t)((Wire.read() << 8) | Wire.read());
      Serial.print(F("  RAW ax ay az=")); Serial.print(v[0]); Serial.print(' '); Serial.print(v[1]);
      Serial.print(' '); Serial.print(v[2]);
      Serial.print(F("  gx gy gz=")); Serial.print(v[4]); Serial.print(' '); Serial.print(v[5]);
      Serial.print(' '); Serial.println(v[6]);
      delay(200);
    }
  }
  Serial.println(F("DONE"));
}

void setup() {
  for (uint8_t i = 0; i < 8; i++) { pinMode(MOTOR_PINS[i], OUTPUT); digitalWrite(MOTOR_PINS[i], LOW); }
  Serial.begin(115200);
  delay(300);
  Serial.println(F("IMU PROBE"));
  probe();
}

void loop() {
  delay(3000);
  Serial.println(F("--- again ---"));
  probe();
}
