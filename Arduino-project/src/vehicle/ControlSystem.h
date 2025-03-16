#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <vector>

class ControlSystem{
    public:
        static void init();
        static std::vector<int> update(double velocity1, double velocity2, double desiredRadius, double desiredVelocity);
        
        static long long lastTime;

        static const double WHEEL_RADIUS = 3.25;
        static const double WHEEL_DISTANCE = 21.5;

        static double velocity1LastError;
        static double velocity2LastError;
        static double velocity1Integral;
        static double velocity2Integral;
        static double velocity1Derivative;
        static double velocity2Derivative;

        static double velocityKp;
        static double velocityKi;
        static double velocityKd;
        static double velocityIntegralLimit;
        static double velocityDerivativeLimit;
};

#endif // CONTROL_SYSTEM_H