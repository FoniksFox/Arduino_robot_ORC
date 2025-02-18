#include "DistanceSensor.h"
#include <Arduino.h>

DistanceSensor::DistanceSensor(int trigPin, int echoPin) : trigPin(trigPin), echoPin(echoPin) {}

void DistanceSensor::init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

long DistanceSensor::getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;
    
    return distance;
}