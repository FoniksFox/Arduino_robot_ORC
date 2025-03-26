#ifndef VEHICLE_H
#define VEHICLE_H

#include "interfaces/InterfaceVehicle.h"
#include "ControlSystem.h"
#include "../components//DistanceSensor/DistanceSensor.h"
#include "../components/MotorController/MotorController.h"
#include "../components/Motor/Motor.h"
#include "../components/LineSensor/LineSensor.h"
#include "../components/VelocitySensor/VelocitySensor.h"
#include "../bluetooth/Bluetooth.h"

#include "../maze/Scout.h"
#include "../maze/PathFind.h"

class Vehicle : public IVehicle, public Bluetooth {
    public:
        Vehicle();
        void init() override;
        void update() override;
        void processOrder(StaticJsonDocument<200> doc) override;
        
        DistanceSensor getDistanceSensor();
        MotorController getMotorController();
        Motor getMotor1();
        Motor getMotor2();
        LineSensor getLineSensor();
        VelocitySensor getVelocitySensor1();
        VelocitySensor getVelocitySensor2();

    private:
        void moveForwardDistance(double velocity, unsigned long duration);
        bool checkWall(int checkDirection);
        void updatePosition();
        void navigateToTarget(int targetX, int targetY);
        std::string generatePathString();

        DistanceSensor distanceSensor;
        MotorController motorController;
        Motor motor1;
        Motor motor2;
        LineSensor lineSensor;
        VelocitySensor velocitySensor1;
        VelocitySensor velocitySensor2;
        PathFinder pathFinder;
        Scout mazeScout;

        double rightAnglePoint;
        double angleSensibility;
        double velocitySensibility;
        double distanceSensibility;

        long long lastUpdateTime;
        double direction;
        double desiredDirection;
        double velocity;
        double desiredVelocity;
        int mode;

        // Vehicle specific variables
        short lastLine;

        // Obstacles course specific variables
        short line;

        // Maze solver specific variables
        short repetition;
        short maze[49][49]; // Wheights matrix for the 7x7 maze, takes corners and straight lines into account
        std::vector<short> mazeSolution; // Solution vector for the maze
        short mazeSolutionIndex; // Index for the solution vector
        short mazeX;
        short mazeY;
        short mazeDirection;
        std::string nextMove;
        std::pair<int, int> currentPos;
        int currentDirection;
        std::vector<std::vector<std::vector<int>>> mazeData;
        const int MAZE_SIZE = 7;

};

#endif // VEHICLE_H