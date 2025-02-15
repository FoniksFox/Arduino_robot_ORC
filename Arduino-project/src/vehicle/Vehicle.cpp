#include "Vehicle.h"

int sensors[8] = {10, 9, 8, 7, 6, 5, 4, 3};

Vehicle::Vehicle() : 
    controlSystem(),
    distanceSensor(15, 18), 
    motorController(14, 27, 21, 22, 12, 25), 
    motor1(motorController, 1), 
    motor2(motorController, 2), 
    lineSensor(11, sensors), 
    velocitySensor1(31), 
    velocitySensor2(30) {}

void Vehicle::init() {
    distanceSensor.init();
    motor1.init();
    motor2.init();
    lineSensor.init();
    velocitySensor1.init();
    velocitySensor2.init();
}

void Vehicle::update() {
    // Update logic here
}

ControlSystem Vehicle::getControlSystem() {
    return controlSystem;
}

DistanceSensor Vehicle::getDistanceSensor() {
    return distanceSensor;
}

MotorController Vehicle::getMotorController() {
    return motorController;
}

Motor Vehicle::getMotor1() {
    return motor1;
}

Motor Vehicle::getMotor2() {
    return motor2;
}

LineSensor Vehicle::getLineSensor() {
    return lineSensor;
}

VelocitySensor Vehicle::getVelocitySensor1() {
    return velocitySensor1;
}

VelocitySensor Vehicle::getVelocitySensor2() {
    return velocitySensor2;
}
