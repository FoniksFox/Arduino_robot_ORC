#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController(int enA, int enB, int in1, int in2, int in3, int in4) : 
    enA(enA), enB(enB), in1(in1), in2(in2), in3(in3), in4(in4) {}

void MotorController::init() {
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void MotorController::setMotorSpeed(int motor, int speed) {
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;
    if (motor == 1) {
        digitalWrite(in1, speed > 0 ? HIGH : LOW);
        digitalWrite(in2, speed > 0 ? LOW : HIGH);
        if (speed1 - speed > 50) {
            analogWrite(enA, 50);
            delay(100);
            if (speed1 - speed > 100) {
                analogWrite(enA, 100);
                delay(100);
            }
        }
        if (speed1 - speed < -50) {
            analogWrite(enA, 50);
            delay(100);
            if (speed1 - speed < -100) {
                analogWrite(enA, 100);
                delay(100);
            }
        }
        analogWrite(enA, speed > 0 ? speed : -speed);
        speed1 = speed;
    } else if (motor == 2) {
        digitalWrite(in3, speed > 0 ? HIGH : LOW);
        digitalWrite(in4, speed > 0 ? LOW : HIGH);
        if (speed2 - speed > 50) {
            analogWrite(enB, 50);
            delay(100);
            if (speed2 - speed > 100) {
                analogWrite(enA, 100);
                delay(100);
            }
        }
        if (speed2 - speed < -50) {
            analogWrite(enA, 50);
            delay(100);
            if (speed2 - speed < -100) {
                analogWrite(enA, 100);
                delay(100);
            }
        }
        analogWrite(enA, speed > 0 ? speed : -speed);
        speed2 = speed;
    }
}

int MotorController::getMotorSpeed(int motor) {
    if (motor == 1) {
        return speed1;
    } else if (motor == 2) {
        return speed2;
    }
}

void MotorController::stopMotor(int motor) {
    setMotorSpeed(motor, 0);
}