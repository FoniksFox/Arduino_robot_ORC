#include <Arduino.h>
#include "../src/vehicle/Vehicle.h"

// Components initialization
Vehicle vehicle;
#define BATTERY_PIN 34  
#define BATTERY_MAX 4200 
#define BATTERY_MIN 3200  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(BATTERY_PIN, INPUT);
  Serial.println("Initializing vehicle");
  vehicle.init();
  Serial.println("Vehicle initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Updating vehicle");
  vehicle.update();
  delay(1000);
}