#ifndef IDISTANCESENSOR_H
#define IDISTANCESENSOR_H

class IDistanceSensor {
    public:
        virtual void init() = 0;
        virtual long getDistance() = 0;

};
#endif // IDISTANCESENSOR_H