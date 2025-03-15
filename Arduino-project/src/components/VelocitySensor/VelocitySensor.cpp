#include "VelocitySensor.h"
#include <Arduino.h>
#include <cmath>

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
    unsigned long elapsedTime = (currentTime - lastTime) / 1000; // Convert to seconds

    if (elapsedTime >= 100) { // Update every tenth of a second
        noInterrupts();
        int count = pulseCount;
        pulseCount = 0;
        interrupts();

        velocity = count / 25 * PI * 6.4 / elapsedTime; // Calculate cm/s
        lastTime = currentTime;
    }
}

void VelocitySensor::handleInterrupt() {
    if (instance) {
        instance->pulseCount++;
    }
}