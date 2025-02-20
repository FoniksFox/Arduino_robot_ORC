#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor (int IR, int sensors[8]) : IR(IR) {
    for(int i = 0; i < 8; i++) {
        this->sensors[i] = sensors[i];
    }
}

void LineSensor::init() {
    pinMode(IR, OUTPUT);
    for(int i = 0; i < 8; i++) {
        pinMode(sensors[i], INPUT);
    }
}

double LineSensor::getLinePosition() { // Returns values between -3 and 3, excepcionally -10 for no line and 10 for intersection
    int position = 0;
    int sum = 0;

    for(int i = 0; i < 8; i++) {
        int value = digitalRead(sensors[i]);
        sum += value;
        position += value * (i - 3);
    }

    if(sum == 0) {
        return 10; // Intersection
    } else if(sum == 8) {
        return -10; // No line
    }
    return position / sum;
}