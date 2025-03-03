#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <vector>

class ControlSystem{
    public:
        static void init();
        static std::vector<int> update(double velocity1, double velocity2, double position, double distance, double desiredVelocity);

    private:
        static double positionError;
        static double positionProportionalError;
        static double positionIntegral;
        static double positionDerivative;
        static double positionKp;
        static double positionKi;
        static double positionKd;

        static double distanceError;
        static double distanceKp;
        static double distanceKd;
        static double lastDistance;

        static double Kvelocity;
        static double Kposition;
        static double Kdistance;

        static double INTEGRAL_LIMIT;
        static long long lastTime;
};

#endif // CONTROL_SYSTEM_H