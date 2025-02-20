#ifndef ICONTROLSYSTEM_H
#define ICONTROLSYSTEM_H

#include <vector>

class IControlSystem {
    public:
        virtual void init() = 0;
        virtual std::vector<int> update(double velocity1, double velocity2, int theoreticalVelocity1, int theoreticalVelocity2, double position, double distance) = 0;
};

#endif // ICONTROLSYSTEM_H