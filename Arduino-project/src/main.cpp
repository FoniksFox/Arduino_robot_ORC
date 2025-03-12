#include <Arduino.h>
#include "../src/vehicle/Vehicle.h"

// Components initialization
Vehicle vehicle;
long long lastTime = millis();
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
  
  // Direct battery reading for debugging
  int rawValue = analogRead(BATTERY_PIN);
  float voltage = rawValue * 3.3 / 4095.0;
  int batteryMv = int(voltage * 1000.0); 
  int batteryPercentage = map(batteryMv, BATTERY_MIN, BATTERY_MAX, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  Serial.print("Direct battery reading - Raw: ");
  Serial.print(rawValue);
  Serial.print(", Voltage: ");
  Serial.print(voltage);
  Serial.print("V, Percentage: ");
  Serial.print(batteryPercentage);
  Serial.println("%");
  if (lastTime + 100 > millis()) delay(lastTime + 100 - millis());
  lastTime = millis();
}