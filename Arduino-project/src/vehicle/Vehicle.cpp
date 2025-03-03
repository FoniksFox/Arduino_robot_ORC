#include "Vehicle.h"
#include <vector>
#include <ArduinoJson.h>

int sensors[8] = {26, 25, 33, 32, 35, 34, 39, 36};

Vehicle::Vehicle() : 
    distanceSensor(13, 18), 
    motorController(14, 19, 23, 22, 12, 21), 
    motor1(motorController, 1), 
    motor2(motorController, 2), 
    lineSensor(27, sensors), 
    velocitySensor1(16), 
    velocitySensor2(17) 
{}

void Vehicle::init() {
    distanceSensor.init();
    motorController.init();
    motor1.init();
    motor2.init();
    lineSensor.init();
    velocitySensor1.init();
    velocitySensor2.init();
    ControlSystem::init();
    
    deviceName = "ESP32-Robot";
    SERVICE_UUID = "d7aa9e26-3527-416a-aaee-c7b1454642dd";
    CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    deviceConnected = false;
    oldDeviceConnected = false;
    lastUpdateTime = 0;
    connectionRetryDelay = 500;
    reconnectionAttempts = 0;
    commandCallback = nullptr;
    begin();

    velocity = 0;
    direction = 0;
    mode = 0;

    line = 0;
    desiredDirection = 0;
    desiredVelocity = 0;

    waitForConnection();
}

void Vehicle::update() {
    velocity = (velocitySensor1.getVelocity() + velocitySensor2.getVelocity()) / 2;
    direction = direction + (velocitySensor1.getVelocity() - velocitySensor2.getVelocity()) / 2;
    
    std::vector<int> controlState = {0, 0};
    switch (mode) {
        case 0: // Wait still
            break;
        case 1: // Velocity
            controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), lineSensor.getLinePosition(), 1000, 255);
            if (distanceSensor.getDistance() < 10) {
                mode = 0;
            }
            break;
        case 2: // Obstacles course
            if (distanceSensor.getDistance() < 20) {
                if (line = 0) {
                    controlState = {255, -255};
                    line = 1;
                } else {
                    controlState = {-255, 255};
                    line = 0;
                }
                motor1.setSpeed(controlState[0]);
                motor2.setSpeed(controlState[1]);
                delay(1000);
            } else {
                controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), lineSensor.getLinePosition(), distanceSensor.getDistance(), 255);
            }
            break;
        case 3: // Maze solver
            // Implement maze solver
            break;
        case 4: // Football / Manual control
            controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), desiredDirection - direction, 1000, desiredVelocity);
            break;
        default:
            break;
    }

    motor1.setSpeed(controlState[0]);
    motor2.setSpeed(controlState[1]);

    processQueue();
    // Later add logging
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

void Vehicle::processOrder(StaticJsonDocument<200> doc) {
    // Example orders
    int command = doc["command"];
    switch (command) {
        case 0: // Stop
            mode = 0;
            break;
        case 1: // Start
            mode = 1;
            break;
        case 2: // Obstacles course
            mode = 2;
            break;
        default:
            break;
    }

}
