#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "interfaces/InterfaceControlSystem.h"
#include <vector>

class ControlSystem : public IControlSystem {
    public:
        void init() override;
        std::vector<int> update(double velocity1, double velocity2, int theoreticalVelocity1, int theoreticalVelocity2, double position, double distance) override;

    private:
        static double positionError;
        static double positionProportionalError;
        static double positionIntegral;
        static double positionDerivative;
        static double positionKp;
        static double positionKi;
        static double positionKd;

        static double distanceKp;
        static double distanceKd;
        static double lastDistance;

        static double velocityError1;
        static double velocityProportionalError1;
        static double velocityIntegral1;
        static double velocityDerivative1;
        static double velocityKp1;
        static double velocityKi1;
        static double velocityKd1;
        static double velocityError2;
        static double velocityProportionalError2;
        static double velocityIntegral2;
        static double velocityDerivative2;
        static double velocityKp2;
        static double velocityKi2;
        static double velocityKd2;

        static double INTEGRAL_LIMIT;
        double lastTime = 0;
};

#endif // CONTROL_SYSTEM_H