#ifndef MotorController_H
#define MotorController_H

#include "interfaces/InterfaceMotorController.h"

class MotorController : public IMotorController {
    public:
        MotorController(int enA, int enB, int in1, int in2, int in3, int in4);
        void init() override;
        void setMotorSpeed(int motor, int speed) override;
        void stopMotor(int motor) override;

    private:
        int enA;
        int in1;
        int in2;
        int in3;
        int in4;
        int enB;
    };

#endif // MotorController_H