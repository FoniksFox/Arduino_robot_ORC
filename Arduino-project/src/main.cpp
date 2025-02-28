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
	Serial.begin(9600);
  	// put your setup code here, to run once:
  	distanceSensor.init();
	motorController.init();
  	motor1.init();
  	motor2.init();
  	lineSensor.init();
  	velocitySensor1.init();
  	velocitySensor2.init();
}

void loop() {
 	Serial.println("Running");

	delay(1000);
}