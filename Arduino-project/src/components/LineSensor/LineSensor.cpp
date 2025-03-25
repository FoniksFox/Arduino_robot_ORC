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
        int value = analogRead(sensors[i]);
        //Serial.println("Sensor " + String(i) + ": " + String(value));
        if (value > 4000) {
            value = 1;
        } else {
            value = 0;
        }
        //Serial.println("Value: " + String(value));
        sum += value;
        position += value * (7 - i - 3.5);
    }

    if (sum == 0) {
        return 0; // Intersection
    }
    return (position / sum)/3.5*30; // Normaliza to angle between -30 and 30 degrees
}

bool LineSensor::isLineDetected() {
    for(int i = 0; i < 8; i++) {
        if(analogRead(sensors[i]) > 4000) {
            //Serial.println("Line detected");
            return true;
        }
    }
    //Serial.println("No line detected");
    return false;
}