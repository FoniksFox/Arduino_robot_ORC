#ifndef VELOCITYSENSOR_H
#define VELOCITYSENSOR_H

#include "../interfaces/InterfaceVelocitySensor.h"

class VelocitySensor : public IVelocitySensor {
    public:
        VelocitySensor(int pin);
        void init() override;
        double getVelocity() override;

        volatile int pulseCount = 0;

    private:
        int pin;
        int instanceIndex;
        
        unsigned long lastTime;
        double velocity = 0;

        void updateVelocity();
        //handleInterrupt();
};

#endif // VELOCITYSENSOR_H