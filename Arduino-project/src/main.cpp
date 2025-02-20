#include <Arduino.h>
#include "components/DistanceSensor/DistanceSensor.h"
#include "components/MotorController.h"
#include "components/Motor.h"
#include "components/LineSensor.h"
#include "components/VelocitySensor.h"

// put function declarations here:
int myFunction(int, int);

// Components initialization
DistanceSensor distanceSensor(13, 18);
MotorController motorController(14, 19, 23, 22, 12, 21);
Motor motor1(motorController, 1);
Motor motor2(motorController, 2);
int sensors[8] = {26, 25, 33, 32, 35, 34, 39, 36};
LineSensor lineSensor(27, sensors);
VelocitySensor velocitySensor1(16);
VelocitySensor velocitySensor2(17);

void setup() {
  // put your setup code here, to run once:
  distanceSensor.init();
  motor1.init();
  motor2.init();
  lineSensor.init();
  velocitySensor1.init();
  velocitySensor2.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}