#include <Arduino.h>
#include "../src/vehicle/Vehicle.h"

// Components initialization
Vehicle vehicle;

void setup() {
  // put your setup code here, to run once:
  vehicle.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  vehicle.update();
}