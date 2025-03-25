#include "Vehicle.h"
#include <vector>
#include <ArduinoJson.h>

int sensors[8] = {26, 25, 33, 32, 35, 34, 39, 36};

Vehicle::Vehicle() : 
    distanceSensor(13, 18), 
    motorController(19, 14, 12, 21, 23, 22), 
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
    
    deviceName = "We're thinking";
    SERVICE_UUID = "d7aa9e26-3527-416a-aaee-c7b1454642dd";
    CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    deviceConnected = false;
    oldDeviceConnected = false;
    lastUpdateTime = 0;
    connectionRetryDelay = 500;
    reconnectionAttempts = 0;
    commandCallback = nullptr;
    begin();

    rightAnglePoint = 90;
    angleSensibility = 1;
    velocitySensibility = 0;
    distanceSensibility = 0;

    lastUpdateTime = millis();
    velocity = 0;
    direction = 0;
    mode = 4;

    line = 0;
    desiredDirection = 0;
    desiredVelocity = 0;

    repetition = 0;
    mazeSolutionIndex = 0;
    mazeX = 3;
    mazeY = 0;
    mazeDirection = 0;
    for (int i = 0; i < 49; i++) {
        for (int j = 0; j < 49; j++) {
            if (i==j) maze[i][j] = 0;
            else maze[i][j] = -1;
        }
    }
    mazeSolution = {};

    waitForConnection();
}

void Vehicle::update() {
    long deltaT = millis() - lastUpdateTime;
    if (deltaT < 10) return;
    lastUpdateTime = millis();


    velocity = (velocitySensor1.getVelocity() + velocitySensor2.getVelocity()) / 2;
    double v1 = motor1.getSpeed() > 0 ? velocitySensor1.getVelocity() : -velocitySensor1.getVelocity();
    if (motor1.getSpeed() == 0) v1 = 0;
    double v2 = motor2.getSpeed() > 0 ? velocitySensor2.getVelocity() : -velocitySensor2.getVelocity();
    if (motor2.getSpeed() == 0) v2 = 0;
    direction = direction + (v1 - v2) / 21.5 * (360.0 / (2 * PI)) * (deltaT / 1000.0);
    if (direction > 180) {
        direction = direction - 360;
    } else if (direction < -180) {
        direction = direction + 360;
    }


    double directionError = 0;
    if (mode == 1 || mode == 2) {
        if (lineSensor.isLineDetected()) {
            directionError = lineSensor.getLinePosition();
            if (directionError >= 0) {
                lastLine = 1;
            } else {
                lastLine = -1;
            }
        } else {
            if (lastLine == 1) {
                directionError = 45;
            } else {
                directionError = -45;
            }
        }
        /* Backup bang bang controller
            if (desideredDirection > 0) {
                controlState = {255, 0};
            } else {
                controlState = {0, 255};
            }
        */
    } else if (mode == 4) {
        directionError = desiredDirection;
    }
    if (directionError == 0) {
        directionError = 1;
    }
    double directionRadius = (pow(rightAnglePoint, angleSensibility) * 10.75) / pow(directionError, angleSensibility);
    if (directionError < 0) {
        directionRadius = -directionRadius;
    }


    std::vector<int> controlState = {0, 0};
    switch (mode) {
        case 0: // Wait still
            controlState = {0, 0};
            break;

        case 1: // Velocity
            controlState = ControlSystem::update(v1, v2, directionRadius, velocitySensibility);
            break;

        case 2: // Obstacles course
            if (distanceSensor.getDistance() < distanceSensibility) {
                if (line = 0) {
                    controlState = {255, 100};
                    delay(1000);
                    line = 1;
                    lastLine = -1;
                } else {
                    controlState = {100, 255};
                    delay(1000);
                    line = 0;
                    lastLine = 1;
                }
                motor1.setSpeed(controlState[0]);
                motor2.setSpeed(controlState[1]);
                delay(1000);
            } else {
                controlState = ControlSystem::update(v1, v2, directionRadius, velocitySensibility);
            }
            break;

        case 3: // Maze solver
            // Implement maze solver
            break;

        case 4: // Football / Manual control
            controlState = ControlSystem::update(v1, v2, directionRadius, desiredVelocity);
            break;

        default:
            break;
    }

    motor1.setSpeed(controlState[0]);
    motor2.setSpeed(controlState[1]);

    processQueue();
    processConsoleQueue();
    processBatteryReadings();
    std::string log = "Velocity: " + std::to_string(velocity) + ", Direction: " + std::to_string(direction) + ", Mode: " + std::to_string(mode);
    sendConsoleMessage(log.c_str());
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
    int command = doc["0"].as<int>();
    switch (command) {
        case 0: // Stop vehicle
            mode = 0;
            break;
        case 1: // Set mode
            if (doc["1"] == 3) {
                mode = 3;
                repetition = 0;
            } else if (doc["1"] == 5) {
                mode = 3;
                repetition = 1;
            } else if (doc["1"] == 6) {
                mode = 3;
                repetition = 2;
            } else {
                mode = doc["1"];
            }
            break;
        case 2: // Set constants
            ControlSystem::velocityKp = static_cast<double>(doc["1"]) / 100.0;
            ControlSystem::velocityKi = static_cast<double>(doc["2"]) / 100.0;
            ControlSystem::velocityKd = static_cast<double>(doc["3"]) / 100.0;
            ControlSystem::velocityDerivativeLimit = static_cast<double>(doc["4"]);
            ControlSystem::velocityIntegralLimit = static_cast<double>(doc["5"]);
            rightAnglePoint = static_cast<double>(doc["6"]) / 100.0 * 180.0; // Angle than counts as a right turn
            angleSensibility = static_cast<double>(doc["7"]) / 100,0 * 5.0;
            velocitySensibility = static_cast<double>(doc["8"]) / 100,0 * 150.0;
            distanceSensibility = static_cast<double>(doc["9"]);
            break;
        case 3: {// Joystick
            mode = 4;
            int angle = 90 - int(doc["1"]);
            if (angle > 180) {
                angle = angle - 360;
            } else if (angle < -180) {
                angle = angle + 360;
            }
            if (angle == 0) angle = 1;
            desiredDirection = angle;
            desiredVelocity = double(doc["2"])/255.0*velocitySensibility;
            break;
        }
        case 4: {
            // Reset direction
            direction = 0;
            break;
        } 

        default: {break;}
    }

}
