#include <Arduino.h>
#include <arduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

float X, Y, Z;
float acc;

void setup() {
  Serial.begin(9600);
  CircuitPlayground.begin();
  CircuitPlayground.setAccelRange(LIS3DH_RANGE_8_G);  // Can be 2, 4, or 8
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(" ");

  X = CircuitPlayground.motionX();
  Serial.print("\nAcceleration in X: ");
  Serial.print(X);

  Y = CircuitPlayground.motionY();
  Serial.print("\nAcceleration in Y: ");
  Serial.print(Y);

  Z = CircuitPlayground.motionZ();
  Serial.print("\nAcceleration in Z: ");
  Serial.print(Z);

  acc = sqrt((X*X)+(Y*Y)+(Z*Z));
  Serial.print("\nAcceleration: ");
  Serial.print(acc);
  
  delay(1000);
}
