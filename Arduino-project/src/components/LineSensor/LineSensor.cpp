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

int LineSensor::getLinePosition() {
    int position = 0;
    int sum = 0;
    for(int i = 0; i < 8; i++) {
        int value = digitalRead(sensors[i]);
        sum += value;
        position += value * (i - 3);
    }
    if(sum == 0) {
        return 0;
    }
    return position / sum;
}