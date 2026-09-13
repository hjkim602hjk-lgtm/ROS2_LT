#include <Arduino.h>
// Lift wheels before powering on. Runs ONCE after reset.
// Left: OUT3/4. Right: OUT1/2. Remove ENA/ENB caps for PWM wiring.
const int EN_L=6, L1=9, L2=10;
const int EN_R=5, R1=7, R2=8;
void stopMotors() {
  analogWrite(EN_L, 0);
  analogWrite(EN_R, 0);
}
void setup() {
  const int pins[]={EN_L,L1,L2,EN_R,R1,R2};
  for (int p:pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  digitalWrite(L1, HIGH);
  digitalWrite(R1, HIGH);
  delay(3000);
  analogWrite(EN_L, 120);
  delay(5000);
  stopMotors();
  delay(2000);
  analogWrite(EN_R, 120);
  delay(5000);
  stopMotors();
  delay(2000);
  analogWrite(EN_L, 120);
  analogWrite(EN_R, 120);
  delay(5000);
  stopMotors();
}
void loop() {}
