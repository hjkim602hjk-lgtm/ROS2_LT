#include <Arduino.h>
// Sensor-only USB test. Disconnect the motor battery.
// Left sensor AO -> UNO A2, VCC -> 5V, GND -> GND. DO unused.
// Right sensor and CdS are not used in this sketch.
void setup() {
  Serial.begin(115200);
  pinMode(A2, INPUT);
}
void loop() {
  Serial.print("Left(A2)=");
  Serial.println(analogRead(A2));
  delay(200);
}
