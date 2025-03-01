#include "VelocitySensor.h"
#include <Arduino.h>

VelocitySensor* instance = nullptr;

VelocitySensor::VelocitySensor(int pin) : pin(pin) {
    instance = this;
}

void VelocitySensor::init() {
    pinMode(pin, INPUT);
    int interruptNumber = digitalPinToInterrupt(pin);
    if (interruptNumber == NOT_AN_INTERRUPT) {
        Serial.println("Error: Pin does not support interrupts");
        return;
    } else {
        Serial.println("Interrupt number: " + String(interruptNumber));
        attachInterrupt(interruptNumber, handleInterrupt, RISING);
    }
    
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
        Serial.println("Interrupt\n");
        instance->pulseCount++;
    }
}