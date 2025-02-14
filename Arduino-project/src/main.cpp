#include <Arduino.h>
#include "components/DistanceSensor.h"

// put function declarations here:
int myFunction(int, int);

DistanceSensor distanceSensor(15, 18);

void setup() {
  // put your setup code here, to run once:
  distanceSensor.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}