#include <Arduino.h>
#include "../src/vehicle/Vehicle.h"

// Components initialization
Vehicle vehicle;
long long lastTime = millis();

void setup()
{
  // put your setup code here, to run once:
  pinMode(4, INPUT);
  Serial.begin(9600);
  Serial.println("Initializing vehicle");
  vehicle.init();
  Serial.println("Vehicle initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Updating vehicle");
  vehicle.update();
  pinMode(4, INPUT);
  Serial.println(analogRead(4));
  if (lastTime + 100 > millis()) delay(lastTime + 100 - millis());
  lastTime = millis();
}