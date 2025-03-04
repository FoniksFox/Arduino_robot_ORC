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
    int position = 0;
    int sum = 0;

    for (int i = 0; i < 8; i++) {
        int value = digitalRead(sensors[i]);
        sum += value;
        position += value * (i - 3.5);
    }

    if (sum == 0) {
        return 0; // Intersection
    }
    else if (sum == 8) {
        return -1000; // No line
    }
    return (position / sum)/3.5*45;
}

bool LineSensor::isLineDetected() {
    for(int i = 0; i < 8; i++) {
        if(digitalRead(sensors[i]) == 1) {
            return true;
        }
    }
    return false;
}