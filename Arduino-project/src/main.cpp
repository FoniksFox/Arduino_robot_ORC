#include <Arduino.h>
#include "components/DistanceSensor/DistanceSensor.h"
#include "components/MotorController/MotorController.h"
#include "components/Motor/Motor.h"
#include "components/LineSensor/LineSensor.h"
#include "components/VelocitySensor/VelocitySensor.h"

// Components initialization
DistanceSensor distanceSensor(13, 18);
int sensors[8] = {26, 25, 33, 32, 35, 39, 34, 36};
LineSensor lineSensor(27, sensors);
VelocitySensor velocitySensor1(16);
VelocitySensor velocitySensor2(17);
MotorController motorController(19, 14, 12, 21, 23, 22);
Motor motor1(motorController, 1);
Motor motor2(motorController, 2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  distanceSensor.init();
  lineSensor.init();
  velocitySensor1.init();
  velocitySensor2.init();
  motorController.init();
  motor1.init();
  motor2.init();
  motor1.setSpeed(255);
  motor2.setSpeed(-100);
  pinMode(4, INPUT);
}

void loop() {
  Serial.println("Distance: " + String(distanceSensor.getDistance()));
  Serial.println("Line: " + String(lineSensor.getLinePosition()));
  Serial.println("Velocity 1: " + String(velocitySensor1.getVelocity()));
  Serial.println("Velocity 2: " + String(velocitySensor2.getVelocity()));
  Serial.println("Battery: " + String(analogRead(4)/42) + "%");
  delay(1000);
}