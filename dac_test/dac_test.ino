#include <Arduino.h>
#define MOD_FAILURE 32 // Feedback failure status

void setup() {
  analogWriteResolution(12); // use 12-bit DAC resolution on Due
  analogWrite(DAC0, 2048);
  pinMode(2, OUTPUT);
  digitalWrite(32, HIGH);
}

void loop() {
  for (int v = 2048; v >= 1950; v -= 1) {
    analogWrite(DAC0, 0);
    digitalWrite(MOD_FAILURE, LOW);
    delay(30);
  }
  for (int v = 1950; v <= 2048; v += 1) {
    analogWrite(DAC0, 2048);
    digitalWrite(MOD_FAILURE, HIGH);
    delay(30);
  }
  
}