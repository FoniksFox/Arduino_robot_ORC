#ifndef VEHICLE_H
#define VEHICLE_H

#include "interfaces/InterfaceVehicle.h"
#include "ControlSystem.h"
#include "../components/DistanceSensor.h"
#include "../components/MotorController.h"
#include "../components/Motor.h"
#include "../components/LineSensor.h"
#include "../components/VelocitySensor.h"

class Vehicle : public IVehicle {
    public:
        Vehicle();
        void init() override;
        void update() override;
        ControlSystem getControlSystem();
        DistanceSensor getDistanceSensor();
        MotorController getMotorController();
        Motor getMotor1();
        Motor getMotor2();
        LineSensor getLineSensor();
        VelocitySensor getVelocitySensor1();
        VelocitySensor getVelocitySensor2();

    private:
        ControlSystem controlSystem;
        DistanceSensor distanceSensor;
        MotorController motorController;
        Motor motor1;
        Motor motor2;
        LineSensor lineSensor;
        VelocitySensor velocitySensor1;
        VelocitySensor velocitySensor2;
};

#endif // VEHICLE_H