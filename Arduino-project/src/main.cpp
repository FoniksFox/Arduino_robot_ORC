#include <Arduino.h>
#include "components/DistanceSensor/DistanceSensor.h"
#include "components/MotorController/MotorController.h"
#include "components/Motor/Motor.h"
#include "components/LineSensor/LineSensor.h"
#include "components/VelocitySensor/VelocitySensor.h"
#include "bluetooth/Bluetooth.h"

#include <ArduinoJson.h>

Bluetooth bluetooth;

void setup() {
    Serial.begin(9600);
    bluetooth.begin();
    
}

void loop() {
	// Code logic



	// Bluetooth handling
	StaticJsonDocument<200> doc;
	bluetooth.processQueue();
	delay(50);
}

