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
    ControlSystem::init();

    velocity = 0;
    direction = 0;
    mode = 0;
}

void Vehicle::update() {
    velocity = (velocitySensor1.getVelocity() + velocitySensor2.getVelocity()) / 2;
    direction = direction + (velocitySensor1.getVelocity() - velocitySensor2.getVelocity()) / 2;
    
    std::vector<int> controlState = {0, 0};
    switch (mode) {
        case 0: // Wait still
            break;
        case 1: // Velocity
            controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), lineSensor.getLinePosition(), 1000);
            if (distanceSensor.getDistance() < 10) {
                mode = 0;
            }
            break;
        case 2: // Obstacles course
            if (distanceSensor.getDistance() < 20) {
                // Change line
            } else {
                controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), lineSensor.getLinePosition(), distanceSensor.getDistance());
            }
            break;
        case 3: // Maze solver
            // Implement maze solver
            break;
        case 4: // Football / Manual control
            // Implement football
            break;
        default:
            break;
    }

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
