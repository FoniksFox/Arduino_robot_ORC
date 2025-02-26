#ifndef IVEHICLE_H
#define IVEHICLE_H

#include "../ControlSystem.h"
#include "../../components/DistanceSensor/DistanceSensor.h"
#include "../../components/MotorController/MotorController.h"
#include "../../components/Motor/Motor.h"
#include "../../components/LineSensor/LineSensor.h"
#include "../../components/VelocitySensor/VelocitySensor.h"

class IVehicle
{
public:
    virtual void init() = 0;
    virtual void update() = 0;

    virtual DistanceSensor getDistanceSensor() = 0;
    virtual MotorController getMotorController() = 0;
    virtual Motor getMotor1() = 0;
    virtual Motor getMotor2() = 0;
    virtual LineSensor getLineSensor() = 0;
    virtual VelocitySensor getVelocitySensor1() = 0;
    virtual VelocitySensor getVelocitySensor2() = 0;
};

#endif // IVEHICLE_H