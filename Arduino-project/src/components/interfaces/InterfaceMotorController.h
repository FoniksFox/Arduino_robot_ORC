#ifndef IMOTORCONTROLLER.H
#define IMOTORCONTROLLER.H

class IMotorController {
    public:
        virtual void init() = 0;
        virtual void setMotorSpeed(int motor, int speed) = 0;
        virtual void stopMotor(int motor) = 0;
};

#endif // IMOTORCONTROLLER.H