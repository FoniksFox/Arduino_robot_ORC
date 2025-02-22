#ifndef VEHICLE_H
#define VEHICLE_H

#include "interfaces/InterfaceVehicle.h"
#include "ControlSystem.h"
#include "../components//DistanceSensor/DistanceSensor.h"
#include "../components/MotorController/MotorController.h"
#include "../components/Motor/Motor.h"
#include "../components/LineSensor/LineSensor.h"
#include "../components/VelocitySensor/VelocitySensor.h"

class Vehicle : public IVehicle {
    public:
        Vehicle();
        void init() override;
        void update() override;
        void setDesiredState(double velocity, double rotation, double distance) override;
        
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

        double desiredVelocity = 0;
        double desiredRotation = 0;
        double desiredDistance = 0;
};

#endif // VEHICLE_H