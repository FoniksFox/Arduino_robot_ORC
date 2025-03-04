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
    velocity = (velocitySensor1.getVelocity() + velocitySensor2.getVelocity()) / 2;
    direction = direction + (velocitySensor1.getVelocity() - velocitySensor2.getVelocity()) / 2;
    if (direction > 180) {
        direction = direction - 360;
    } else if (direction < -180) {
        direction = direction + 360;
    }
    
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
            if (repetition == 0) {
                // Maze recognition, run dfs
                if (distanceSensor.getDistance() < 10) {
                    // Wall ahead
                    if (direction == 0) {
                        maze[7*mazeY + mazeX][7*(mazeY-1) + mazeX] = 1;
                    } else if (direction == 90) {
                        maze[7*mazeY + mazeX][7*mazeY* + (mazeX+1)] = 2;
                    } else if (direction == 180) {
                        maze[7*mazeY + mazeX][7*(mazeY+1) + mazeX] = 1;
                    } else if (direction == 270) {
                        maze[7*mazeY + mazeX][7*mazeY* + (mazeX-1)] = 2;
                    }
                }
                // Check next direction
                if (maze[7*mazeY + mazeX][7*(mazeY-1) + mazeX] == -1) {
                    desiredDirection = 0;
                } else if (maze[7*mazeY + mazeX][7*mazeY* + (mazeX+1)] == -1) {
                    desiredDirection = 90;
                } else if (maze[7*mazeY + mazeX][7*(mazeY+1) + mazeX] == -1) {
                    desiredDirection = 180;
                } else if (maze[7*mazeY + mazeX][7*mazeY* + (mazeX-1)] == -1) {
                    desiredDirection = 270;
                } else {
                    // Run dfs

                }
            } else if (repetition == 1) {
                // Run maze with lax speed
            } else if (repetition == 2) {
                // Run maze with high speed
            }
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
    int command = doc[0];
    switch (command) {
        case 0: // Stop vehicle
            mode = 0;
            break;
        case 1: // Set mode
            mode = doc[1];
            break;
        case 2: // Set constants
            ControlSystem::positionKp = doc[1] / 100.0;
            ControlSystem::positionKi = doc[2] / 100.0;
            ControlSystem::positionKd = doc[3] / 100.0;
            ControlSystem::distanceKp = doc[4] / 100.0;
            ControlSystem::distanceKd = doc[5] / 100.0;
            ControlSystem::Kvelocity = doc[6] / 100.0;
            ControlSystem::Kposition = doc[7] / 100.0;
            ControlSystem::Kdistance = doc[8] / 100.0;
            ControlSystem::INTEGRAL_LIMIT = doc[9];
            break;
        case 3: // Joystick
            int angle = doc[1];
            if (angle > 180) {
                angle = angle - 360;
            } else if (angle < -180) {
                angle = angle + 360;
            }
            desiredDirection = angle;
            desiredVelocity = doc[2];
            break;
        case 4: // Reset direction
            direction = 0;
            break;
        default:
            break;
    }

}
