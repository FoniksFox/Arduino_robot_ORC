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
    if (deltaT < 100) {
        return;
    }
    lastUpdateTime = millis();
    velocity = (velocitySensor1.getVelocity() + velocitySensor2.getVelocity()) / 2;
    direction = direction + (velocitySensor1.getVelocity() - velocitySensor2.getVelocity()) / (lastUpdateTime / 1000) / 21.5 * (180 / PI);
    if (direction > 180) {
        direction = direction - 360;
    } else if (direction < -180) {
        direction = direction + 360;
    }
    std::vector<int> controlState = {0, 0};

    Serial.println("Direction : " + String(direction));
    Serial.println("Velocity : " + String(velocity));

    double directionError = 0;
    switch (mode) {
        case 0: // Wait still
            Serial.println("Nothing");
            break;

        case 1: // Velocity
            Serial.println("Velocity Challenge");
            controlState = ControlSystem::update(velocitySensor1.getVelocity(), velocitySensor2.getVelocity(), lineSensor.getLinePosition(), 1000, 255);
            if (distanceSensor.getDistance() < 10) {
                mode = 0;
            }
            break;

        case 2: // Obstacles course
            Serial.println("Obstacle Course");
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
            Serial.println("Maze Solver");
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
            Serial.println("Manual control");
            Serial.println("Desired Direction: " + String(desiredDirection) + ", Desired Velocity: " + String(desiredVelocity));
            directionError = desiredDirection - direction;
            if (directionError > 180) {
                directionError = directionError - 360;
            } else if (directionError < -180) {
                directionError = directionError + 360;
            }
            controlState = ControlSystem::update(velocitySensor1.getVelocity() - motor1.getSpeed(), velocitySensor2.getVelocity() - motor2.getSpeed(), directionError, 1000, desiredVelocity);
            break;

        default:
            break;
    }

    Serial.println("Control State: " + String(controlState[0]) + ", " + String(controlState[1]));
    motor1.setSpeed(controlState[0]);
    motor2.setSpeed(controlState[1]);

    processQueue();
    processConsoleQueue();
    processBatteryReadings();
    // Later add logging
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
    // Example orders
    int command = doc["0"].as<int>();
    //Serial.println(command);
    switch (command) {
        case 0: // Stop vehicle
            //Serial.println("Vehicle stopped...");
            mode = 0;
            break;
        case 1: // Set mode
            //Serial.println("Changing Mode...");
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
            //Serial.println("Constants Changed");
            ControlSystem::positionKp = static_cast<double>(doc["1"]) / 100.0;
            ControlSystem::positionKi = static_cast<double>(doc["2"]) / 100.0;
            ControlSystem::positionKd = static_cast<double>(doc["3"]) / 100.0;
            ControlSystem::distanceKp = static_cast<double>(doc["4"]) / 100.0;
            ControlSystem::distanceKd = static_cast<double>(doc["5"]) / 100.0;
            ControlSystem::Kvelocity = static_cast<double>(doc["6"]) / 100.0;
            ControlSystem::Kposition = static_cast<double>(doc["7"]) / 100.0;
            ControlSystem::Kdistance = static_cast<double>(doc["8"]) / 100.0;
            ControlSystem::INTEGRAL_LIMIT = static_cast<int>(doc["9"]);
            break;
        case 3: {// Joystick
            //Serial.println("Joystick moved");
            mode = 4;
            int angle = 90 - int(doc["1"]);
            if (angle > 180) {
                angle = angle - 360;
            } else if (angle < -180) {
                angle = angle + 360;
            }
            desiredDirection = angle;
            desiredVelocity = int(doc["2"]);
            //Serial.println("Direction: " + String(desiredDirection) + ", Velocity: " + String(desiredVelocity));
            break;
        }
        case 4: {
            //Serial.println("Set new North");
            // Reset direction
            direction = 0;
            break;
        } 

        default: {break;}
    }

}
