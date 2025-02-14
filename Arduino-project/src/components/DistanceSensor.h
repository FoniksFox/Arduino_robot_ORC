#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include "interfaces/InterfaceDistanceSensor.h"

class DistanceSensor : public IDistanceSensor {
    public:
        DistanceSensor(int trigPin, int echoPin);
        void init() override;
        long getDistance() override;

    private:
        int trigPin;
        int echoPin;
};

#endif // DISTANCESENSOR_H