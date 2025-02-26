#include <Arduino.h>
#include "../src/vehicle/Vehicle.h"

// put function declarations here:
int myFunction(int, int);

// Components initialization
Vehicle vehicle;

void setup()
{
  // put your setup code here, to run once:
  vehicle.init();
}

void loop()
{
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}