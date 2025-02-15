#ifndef IVELOCITYSENSOR_H
#define IVELOCITYSENSOR_H

class IVelocitySensor {
    public:
        virtual void init() = 0;
        virtual double getVelocity() = 0;
};

#endif // IVELOCITYSENSOR_H