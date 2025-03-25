#include <Arduino.h>
#include "../src/vehicle/Vehicle.h"

// Components initialization
Vehicle vehicle;
unsigned long lastTime = millis();

void setup() {
  pinMode(4, INPUT);
  Serial.begin(9600);
  Serial.println("Initializing vehicle");
  vehicle.init();
  Serial.println("Vehicle initialized");
}

void loop() {
  vehicle.update();
  unsigned long currentTime = millis();
  if (currentTime - lastTime < 10) delay(10 - (currentTime - lastTime));
  currentTime = millis();
  //Serial.println("Time spent: " + String(currentTime - lastTime));
  lastTime = millis();
}