#ifndef IMOTOR_H
#define IMOTOR_H

class IMotor {
    public:
        virtual void init() = 0;
        virtual void setSpeed(int speed) = 0;
        virtual int getSpeed() = 0;
        virtual void stop() = 0;
};

#endif // IMOTOR_H