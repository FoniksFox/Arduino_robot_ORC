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
  //Serial.println("Updating vehicle");
  vehicle.update();
  //pinMode(4, INPUT);
  //Serial.println(analogRead(4));
  unsigned long currentTime = millis();
  if (currentTime - lastTime < 10) delay(10 - (currentTime - lastTime));
  currentTime = millis();
  //Serial.println("Time spent: " + String(currentTime - lastTime));
  lastTime = millis();
}