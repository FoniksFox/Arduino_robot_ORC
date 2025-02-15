#ifndef ICONTROLSYSTEM_H
#define ICONTROLSYSTEM_H

#include <vector>

class IControlSystem {
    public:
        virtual void init() = 0;
        virtual std::vector<double> update(std::vector<double> currentState, std::vector<double> desiredState) = 0;
};

#endif // ICONTROLSYSTEM_H