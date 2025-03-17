#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int IR, int sensors[8]) : IR(IR) {
    for (int i = 0; i < 8; i++)
    {
        this->sensors[i] = sensors[i];
    }
}

void LineSensor::init() {
    pinMode(IR, OUTPUT);
    digitalWrite(IR, HIGH);
    for (int i = 0; i < 8; i++) {
        pinMode(sensors[i], INPUT);
    }
}

double LineSensor::getLinePosition() { // Returns values between -45 and 45 degrees, 0 is the center, -1000 is no line
    double position = 0;
    int sum = 0;

    for (int i = 0; i < 8; i++) {
        double value = analogRead(sensors[i]);
        //Serial.println("Sensor " + String(i) + ": " + String(value));
        value = 4095 - value;
        value /= 4095;
        //Serial.println("Value: " + String(value));
        position += value * (7 - i - 3.5);
    }

    return position/3.5*20;
}

bool LineSensor::isLineDetected() {
    for(int i = 0; i < 8; i++) {
        if(analogRead(sensors[i]) < 1000) {
            return true;
        }
    }
    return false;
}