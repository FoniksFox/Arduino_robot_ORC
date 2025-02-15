#include <Arduino.h>
#include "components/DistanceSensor.h"
#include "components/MotorController.h"
#include "components/Motor.h"
#include "components/LineSensor.h"

// put function declarations here:
int myFunction(int, int);

// Components initialization
DistanceSensor distanceSensor(15, 18);
MotorController motorController(14, 27, 21, 22, 12, 25);
Motor motor1(motorController, 1);
Motor motor2(motorController, 2);
int sensors[8] = {10, 9, 8, 7, 6, 5, 4, 3};
LineSensor lineSensor(11, sensors);

void setup() {
  // put your setup code here, to run once:
  distanceSensor.init();
  motor1.init();
  motor2.init();
  lineSensor.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}