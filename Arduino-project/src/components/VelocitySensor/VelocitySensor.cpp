#include "VelocitySensor.h"
#include <Arduino.h>
#include <cmath>

#define MAX_SENSORS 10

static VelocitySensor* instances[MAX_SENSORS] = {nullptr};

void interrupt1() {
    instances[0]->pulseCount++;
}
void interrupt2() {
    instances[1]->pulseCount++;
}

VelocitySensor::VelocitySensor(int pin) : pin(pin) {
    for (int i = 0; i < MAX_SENSORS; ++i) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            instanceIndex = i;
            break;
        }
    }
}

void VelocitySensor::init() {
    pinMode(pin, INPUT);
    int interruptNumber = digitalPinToInterrupt(pin);
    if (interruptNumber == NOT_AN_INTERRUPT) {
        Serial.println("Error: Pin does not support interrupts");
        return;
    } else {
        Serial.println("Interrupt number: " + String(interruptNumber));
        if (instanceIndex == 0) {
            attachInterrupt(interruptNumber, interrupt1, RISING);
        } else {
            attachInterrupt(interruptNumber, interrupt2, RISING);
        }
    }
    
    lastTime = millis();
}

double VelocitySensor::getVelocity() {
    updateVelocity();
    return velocity;
}

void VelocitySensor::updateVelocity() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = (currentTime - lastTime);

    if (elapsedTime >= 100) { // Update every tenth of a second
        Serial.println("Pulse count: " + String(pulseCount));
        noInterrupts();
        int count = pulseCount;
        pulseCount = 0; 
        interrupts();

        Serial.println("Count: " + String(count));
        velocity = double(count) / 25.0 * PI * 6.4 / (elapsedTime/1000.0); // Calculate cm/s
        lastTime = currentTime;
    }
}

/*void VelocitySensor::handleInterrupt() {
    for (int i = 0; i < MAX_SENSORS; ++i) {
        if (instances[i] != nullptr && digitalRead(instances[i]->pin) == HIGH) {
            instances[i]->pulseCount++;
        }
    }
}
*/