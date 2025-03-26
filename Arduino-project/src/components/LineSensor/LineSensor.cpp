#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int IR, int sensors[8]) : IR(IR) {
    for (int i = 0; i < 8; i++) {
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

std::vector<int> LineSensor::readSensors() {
    std::vector<int> readings(8, 0);
    for (int i = 0; i < 8; i++) {
        int value = analogRead(sensors[i]);
        readings[i] = (value > 4000) ? 1 : 0;
    }
    return readings;
}

double LineSensor::getLinePosition() {
    double position = 0;
    int sum = 0;
    
    for (int i = 0; i < 8; i++) {
        int value = analogRead(sensors[i]);
        int binaryValue = (value > 4000) ? 1 : 0;
        
        sum += binaryValue;
        position += binaryValue * (7 - i - 3.5);
    }
    
    if (sum == 0) {
        return 0;
    }
    
    return (position / sum) / 3.5 * 45;
}

bool LineSensor::isLineDetected() {
    for (int i = 0; i < 8; i++) {
        if (analogRead(sensors[i]) > 4000) {
            return true;
        }
    }
    return false;
}

bool LineSensor::isIntersection() {
    int activeCount = 0;
    for (int i = 0; i < 8; i++) {
        if (analogRead(sensors[i]) > 4000) {
            activeCount++;
        }
    }
    return activeCount >= 3;
}