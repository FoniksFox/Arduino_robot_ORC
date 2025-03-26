#include "Vehicle.h"
#include <vector>
#include <ArduinoJson.h>

int sensors[8] = {26, 25, 33, 32, 35, 34, 39, 36};

struct Point {
    int x, y;
    Point(int _x = 0, int _y = 0) : 
        x(std::max(0, std::min(_x, 6))), 
        y(std::max(0, std::min(_y, 6))) {} 
};

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

    std::vector<std::vector<std::vector<int>>> aux(MAZE_SIZE, 
        std::vector<std::vector<int>>(MAZE_SIZE, std::vector<int>(4, -1)));
    mazeData = aux;
    nextMove = "";

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
            ControlSystem::update(v1, v2, 100000000, 0);
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
        currentPos = std::make_pair(mazeY, mazeX);
        currentDirection = mazeDirection;
        {
            for (int y = 0; y < MAZE_SIZE; y++) {
                for (int x = 0; x < MAZE_SIZE; x++) {
                    for (int dir = 0; dir < 4; dir++) {
                        bool wallExists = checkWall(dir * 90);
                        mazeData[y][x][dir] = wallExists ? 1 : 0;
                        pathFinder.update_wall_knowledge(y, x, dir, wallExists ? 1 : 0);
                    }
                }
            }
        
            pathFinder.set_knowledge_matrix(mazeData);
            std::pair<int, int> currentPos(mazeY, mazeX);
            int currentDirection = static_cast<int>(direction / 90);
        }
            switch (repetition) {
                case 0: // Exploration phase
                {
                    mazeScout.mark_visited(mazeY, mazeX);
                    double explorationProgress = mazeScout.get_exploration_progress();
                    if (mazeScout.is_fully_explored()) {
                        repetition = 1;
                        break;
                    }
                    
                    // More robust next move selection
                    int wallChecks = 0;
                    int consecutiveWallChecks = 3;
                    int wallDetectionThreshold = 20;
                    
                    for (int check = 0; check < consecutiveWallChecks; check++) {
                        if (distanceSensor.getDistance() < wallDetectionThreshold) {
                            wallChecks++;
                        }
                        delay(10);
                    }
                    
                    auto unvisitedNeighbors = mazeScout.get_unvisited_neighbors(mazeData, currentPos);
                    
                    if (unvisitedNeighbors.empty() || wallChecks >= consecutiveWallChecks / 2) {
                        repetition = 1;
                        break;
                    }
                    
                    nextMove = mazeScout.determine_next_move_dfs(mazeData);
                }
                    break;
                
                case 1: // Path to target
                    {
                        if (mazeScout.is_fully_explored()) {
                            std::pair<int, int> targetPos(0, MAZE_SIZE / 2);
                            std::string targetMove = mazeScout.get_direction_to_target(mazeData, targetPos);
                            
                            int targetDirection = -1;
                            
                            // Convert string move to direction
                            if (targetMove == "north") targetDirection = 0;
                            else if (targetMove == "east") targetDirection = 1;
                            else if (targetMove == "south") targetDirection = 2;
                            else if (targetMove == "west") targetDirection = 3;
                            
                            if (targetDirection != -1) {
                                if (currentDirection != targetDirection) {
                                    nextMove = (currentDirection + 1) % 4 == targetDirection ? "right" : "left";
                                } else {
                                    nextMove = "forward";
                                }
                            }
                        } else {
                            mode = 0;  
                        }
                    }
                    break;
                
                case 2: // Move towards unexplored areas
                    {
                        nextMove = mazeScout.move_towards_unexplored(mazeData);
                    }
                    break;
                
                default:
                    break;
            }

            // Move execution
            if (nextMove == "forward") {
                desiredDirection = currentDirection * 90;
                moveForwardDistance(0.5, 500);

                switch(currentDirection) {
                    case 0: mazeY--; break;
                    case 1: mazeX++; break;
                    case 2: mazeY++; break;
                    case 3: mazeX--; break;
                }
                
                delay(200);
            } else if (nextMove == "right") {
                desiredDirection = fmod(currentDirection * 90 + 90, 360);
                desiredVelocity = 0.0;
            } else if (nextMove == "left") {
                desiredDirection = fmod(currentDirection * 90 - 90 + 360, 360);
                desiredVelocity = 0.0;
            }
        
            [[fallthrough]];
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


void Vehicle::moveForwardDistance(double velocity, unsigned long duration) {
    velocity = std::max(-1.0, std::min(1.0, velocity));
    duration = std::min(duration, 5000UL);
    int previousMode = mode;

    mode = 4;

    desiredDirection = direction;
    desiredVelocity = velocity;

    unsigned long startTime = millis();

    while (millis() - startTime < duration) {
        if (distanceSensor.getDistance() < 10.0) {
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            break;
        }

        int controlSignal = static_cast<int>(velocity * 255);
        motor1.setSpeed(controlSignal);
        motor2.setSpeed(controlSignal);

        delay(10);
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    mode = previousMode;
    desiredVelocity = 0;
}

bool Vehicle::checkWall(int direction) {
    int targetX = mazeX;
    int targetY = mazeY;

    if (direction % 90 != 0) {
        Serial.println("Invalid wall check direction");
        return true;
    }
    direction = (direction + 360) % 360;
    
    // Calculate target coordinates
    switch(direction) {
        case 0:   // North
            targetY = std::max(0, mazeY - 1);
            break;
        case 90:  // East
            targetX = std::min(6, mazeX + 1);
            break;
        case 180: // South
            targetY = std::min(6, mazeY + 1);
            break;
        case 270: // West
            targetX = std::max(0, mazeX - 1);
            break;
    }
    if (targetX < 0 || targetX >= 7 || targetY < 0 || targetY >= 7) {
        return true; 
    }
    
    bool wallDetectedBySensor = distanceSensor.getDistance() < 20.0; // EXAMPLE MIGHT NEED TO CHANGE
    bool wallInMazeRepresentation = maze[7*mazeY + mazeX][7*targetY + targetX] == 1;
    
    return wallDetectedBySensor || wallInMazeRepresentation;
}

void Vehicle::updatePosition() {
    int dir = static_cast<int>(direction);

    if (dir >= -45 && dir < 45) { // North
        mazeY--;
    } else if (dir >= 45 && dir < 135) { // East
        mazeX++;
    } else if (dir >= 135 || dir < -135) { // South
        mazeY++;
    } else if (dir >= -135 && dir < -45) { // West
        mazeX--;
    }
    if (mazeX < 0) mazeX = 0;
    if (mazeX >= 7) mazeX = 6;
    if (mazeY < 0) mazeY = 0;
    if (mazeY >= 7) mazeY = 6;
}

void Vehicle::navigateToTarget(int x, int y) {
    // Clamp coordinates to valid maze range
    x = std::max(0, std::min(6, x));
    y = std::max(0, std::min(6, y));
    
    // Create maze data vector
    std::vector<std::vector<std::vector<int>>> mazeData(7, 
    std::vector<std::vector<int>>(7, std::vector<int>(4, -1)));
    
    // Populate maze data with wall information
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int dir = 0; dir < 4; dir++) {
                bool wallExists = checkWall(dir * 90);
                mazeData[i][j][dir] = wallExists ? 1 : 0;
            }
        }
    }
    
    // Current position and target
    std::pair<int, int> currentPos(mazeY, mazeX);
    std::pair<int, int> targetPos(y, x);
    int currentDirection = static_cast<int>(direction / 90);
    
    // Get next move based on path finding
    std::string nextMove = pathFinder.get_next_move(currentPos, currentDirection, targetPos, mazeData);
    
    if (nextMove == "forward") {
        moveForwardDistance(0.5, 500);
        switch(currentDirection) {
            case 0: mazeY--; break;
            case 1: mazeX++; break;
            case 2: mazeY++; break;
            case 3: mazeX--; break;
        }
    } else if (nextMove == "right") {
        desiredDirection = fmod(currentDirection * 90 + 90, 360);
    } else if (nextMove == "left") {
        desiredDirection = fmod(currentDirection * 90 - 90 + 360, 360);
    }
}
