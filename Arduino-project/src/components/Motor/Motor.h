#ifndef MOTOR_H
#define MOTOR_H

#include "../interfaces/InterfaceMotor.h"
#include "../interfaces/InterfaceMotorController.h"

class Motor : public IMotor {
    public:
        Motor(IMotorController& motorController, int motor);
        void init() override;
        void setSpeed(int speed) override;
        int getSpeed() override;
        void stop() override;

    private:
        IMotorController& motorController;
        int motor;
};

#endif // MOTOR_H