#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

float X, Y, Z;

void setup() {
  Serial.begin(9600);
  CircuitPlayground.begin();
  CircuitPlayground.setAccelRange(LIS3DH_RANGE_8_G);  // Can be 2, 4, or 8
}

void loop() {
  // put your main code here, to run repeatedly:
  X = CircuitPlayground.motionX();
  Serial.println(X);
  Y = CircuitPlayground.motionY();
  Serial.println(Y);
  Z = CircuitPlayground.motionZ();
  Serial.println(Z);
  delay(1000);
}
