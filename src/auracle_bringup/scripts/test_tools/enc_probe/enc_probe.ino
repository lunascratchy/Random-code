// Encoder wire probe for the Auracle Nano. Motors are held off.
// Every 500 ms prints, for each encoder wire: its level now and how many times it changed.
//   R_A=D2  R_B=D4  L_A=D3  L_B=D12
// A wire that never changes while its wheel turns is the broken one.
// Both wires of one encoder stuck = that encoder has no power (5V/GND).

const uint8_t PINS[4] = {2, 4, 3, 12};
const char* NAMES[4] = {"R_A(D2)", "R_B(D4)", "L_A(D3)", "L_B(D12)"};
const uint8_t MOTOR_PINS[8] = {5, 6, 9, 10, 7, 8, A0, A1};

uint8_t last[4];
unsigned long edges[4];
unsigned long lastPrint = 0;

void setup() {
  for (uint8_t i = 0; i < 8; i++) { pinMode(MOTOR_PINS[i], OUTPUT); digitalWrite(MOTOR_PINS[i], LOW); }
  for (uint8_t i = 0; i < 4; i++) { pinMode(PINS[i], INPUT_PULLUP); last[i] = digitalRead(PINS[i]); }
  Serial.begin(115200);
  Serial.println(F("PROBE READY"));
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t v = digitalRead(PINS[i]);
    if (v != last[i]) { edges[i]++; last[i] = v; }
  }
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(NAMES[i]); Serial.print('='); Serial.print(last[i]);
      Serial.print(" edges="); Serial.print(edges[i]); Serial.print("   ");
    }
    Serial.println();
  }
}
