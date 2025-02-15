#ifndef VELOCITYSENSOR_H
#define VELOCITYSENSOR_H

#include "interfaces/InterfaceVelocitySensor.h"

class VelocitySensor : public IVelocitySensor {
    public:
        VelocitySensor(int pin);
        void init() override;
        double getVelocity() override;

    private:
        int pin;
        volatile int pulseCount = 0;
        unsigned long lastTime;
        int velocity = 0;

        void updateVelocity();
        static void handleInterrupt();
};

#endif // VELOCITYSENSOR_H