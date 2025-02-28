#include <Arduino.h>
#include "components/DistanceSensor/DistanceSensor.h"
#include "components/MotorController/MotorController.h"
#include "components/Motor/Motor.h"
#include "components/LineSensor/LineSensor.h"
#include "components/VelocitySensor/VelocitySensor.h"

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
  Serial.begin(9600);
  distanceSensor.init();
  motorController.init();
  motor1.init();
  motor2.init();
  lineSensor.init();
  velocitySensor1.init();
  velocitySensor2.init();
}

void loop() {
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  Serial.println("Distance: " + String(distanceSensor.getDistance()));
  Serial.println("Line: " + String(lineSensor.getLinePosition()));
  Serial.println("Velocity 1: " + String(velocitySensor1.getVelocity()));
  Serial.println("Velocity 2: " + String(velocitySensor2.getVelocity()));
  delay(1000);
  motor1.setSpeed(-255);
  motor2.setSpeed(-255);
  delay(1000);
}