#include "VelocitySensor.h"
#include <Arduino.h>

VelocitySensor* instance = nullptr;

VelocitySensor::VelocitySensor(int pin) : pin(pin) {
    instance = this;
}

void VelocitySensor::init() {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), handleInterrupt, RISING);
    lastTime = millis();
}

double VelocitySensor::getVelocity() {
    updateVelocity();
    return velocity;
}

void VelocitySensor::updateVelocity() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastTime;

    if (elapsedTime >= 100) { // Update every tenth of a second
        noInterrupts();
        int count = pulseCount;
        pulseCount = 0;
        interrupts();

        velocity = (count * 60) / elapsedTime; // Calculate RPM (each pulse is a full rotation)
        lastTime = currentTime;
    }
}

void VelocitySensor::handleInterrupt() {
    if (instance) {
        instance->pulseCount++;
    }
}