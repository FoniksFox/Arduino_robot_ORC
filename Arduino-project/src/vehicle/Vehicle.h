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
        DistanceSensor distanceSensor;
        MotorController motorController;
        Motor motor1;
        Motor motor2;
        LineSensor lineSensor;
        VelocitySensor velocitySensor1;
        VelocitySensor velocitySensor2;

        double direction;
        double velocity;
        int mode;

        // Obstacles course specific variables
        short line;

        // Maze solver specific variables
        short maze[15][15];
        short mazeX;
        short mazeY;
        short mazeDirection;

};

#endif // VEHICLE_H