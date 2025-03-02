#include "Vehicle.h"
#include <vector>

int sensors[8] = {26, 25, 33, 32, 35, 34, 39, 36};

Vehicle::Vehicle() : 
    distanceSensor(13, 18), 
    motorController(14, 19, 23, 22, 12, 21), 
    motor1(motorController, 1), 
    motor2(motorController, 2), 
    lineSensor(11, sensors), 
    velocitySensor1(16), 
    velocitySensor2(17) {}

void Vehicle::init() {
    distanceSensor.init();
    motorController.init();
    motor1.init();
    motor2.init();
    lineSensor.init();
    velocitySensor1.init();
    velocitySensor2.init();
}

void Vehicle::update() {
    double desiredDirection = 0; // Later obtained from bluetooth
    double maxSpeed = 100; // Later obtained from bluetooth
    std::vector<int> controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), direction - desiredDirection, distanceSensor.getDistance(), maxSpeed);

    motor1.setSpeed(controlState[0]);
    motor2.setSpeed(controlState[1]);
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
