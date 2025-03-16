#include "ControlSystem.h"
#include <vector>
#include <algorithm>
#include <Arduino.h>

// Define static member variables
long long ControlSystem::lastTime;

double const ControlSystem::WHEEL_RADIUS;
double const ControlSystem::WHEEL_DISTANCE;

double ControlSystem::velocity1LastError;
double ControlSystem::velocity2LastError;
double ControlSystem::velocity1Integral;
double ControlSystem::velocity2Integral;
double ControlSystem::velocity1Derivative;
double ControlSystem::velocity2Derivative;

double ControlSystem::velocityKp;
double ControlSystem::velocityKi;
double ControlSystem::velocityKd;
double ControlSystem::velocityIntegralLimit;
double ControlSystem::velocityDerivativeLimit;


/* Control System diagram (may need improvements, like kalman filter and maybe a manual bypass):
+-----------------+          ___________           +-----------------+       +-----------------+
|  Theoretical    |         /           \          |   Controller    |       |   Actuators     |
|    State        |      + /             \         |                 |       |                 |
|   (Vector)      +------>|               |------->+                 +------>+                 |
|    [v1, v2]     |        \             /         |   PID Control   |       |     Motors      |
|                 |         \___________/          |   (Vector)      |       |    [v1, v2]     |
+-----------------+               ^ -              +-----------------+       +--------+--------+
                                  |                                                   |
                                  |                                                   |
                                  |                                                   v
                         +--------+--------+                                 +--------+--------+
                         |   Sensors       |                                 |   Vehicle       |
                         |   (Vector)      |                                 |  Dynamics       |
                         | [v1, v2, p, d]  +<--------------------------------+                 |
                         |                 |                                 |                 |
                         +-----------------+                                 +-----------------+
*/

void ControlSystem::init()
{
    
    lastTime = millis();
}

std::vector<int> ControlSystem::update(double velocity1, double velocity2, double desiredRadius, double desiredVelocity) {
    std::vector<int> controlSignal = {0, 0};

    if (lastTime == 0) lastTime = millis();
    double deltaT = millis() - lastTime + 1;  // Avoid division by zero

    // Reference velocities
    double referenceVelocity1 = desiredVelocity * (1 - WHEEL_DISTANCE / (2 * desiredRadius));
    double referenceVelocity2 = desiredVelocity * (1 + WHEEL_DISTANCE / (2 * desiredRadius));

    // Errors
    double velocity1Error = referenceVelocity1 - velocity1;
    double velocity2Error = referenceVelocity2 - velocity2;

    // Compute alpha for rotation priority
    double alpha = std::abs(velocity2Error - velocity1Error) / (std::abs(velocity2Error) + std::abs(velocity1Error) + 1e-6);

    // Integral terms (clamped)
    velocity1Integral = velocity1Integral + velocity1Error * deltaT, -velocityIntegralLimit;
    velocity2Integral = velocity2Integral + velocity2Error * deltaT, -velocityIntegralLimit;
    if (velocity1Integral > velocityIntegralLimit) velocity1Integral = velocityIntegralLimit;
    if (velocity1Integral > -velocityIntegralLimit) velocity1Integral = -velocityIntegralLimit;
    if (velocity2Integral > velocityIntegralLimit) velocity2Integral = velocityIntegralLimit;
    if (velocity2Integral > -velocityIntegralLimit) velocity2Integral = -velocityIntegralLimit;

    // Derivative terms (clamped)
    velocity1Derivative = (velocity1Error - velocity1LastError) / deltaT, -velocityDerivativeLimit;
    velocity2Derivative = (velocity2Error - velocity2LastError) / deltaT, -velocityDerivativeLimit;
    if (velocity1Derivative > velocityDerivativeLimit) velocity1Derivative = velocityDerivativeLimit;
    if (velocity1Derivative > -velocityDerivativeLimit) velocity1Derivative = -velocityDerivativeLimit;
    if (velocity2Derivative > velocityDerivativeLimit) velocity2Derivative = velocityDerivativeLimit;
    if (velocity2Derivative > -velocityDerivativeLimit) velocity2Derivative = -velocityDerivativeLimit;

    // PID Output
    double rawControl1 = velocityKp * velocity1Error + velocityKi * velocity1Integral + velocityKd * velocity1Derivative;
    double rawControl2 = velocityKp * velocity2Error + velocityKi * velocity2Integral + velocityKd * velocity2Derivative;

    // Apply rotation weight (alpha)
    controlSignal[0] = rawControl1 * (1 - alpha);
    controlSignal[1] = rawControl2 * (1 + alpha);

    // Constraints to 255
    int maxControl = std::max(std::abs(controlSignal[0]), std::abs(controlSignal[1]));
    if (maxControl > 255) {
        controlSignal[0] = controlSignal[0] * 255 / maxControl;
        controlSignal[1] = controlSignal[1] * 255 / maxControl;
    }

    // Update previous values
    velocity1LastError = velocity1Error;
    velocity2LastError = velocity2Error;
    lastTime = millis();

    return controlSignal;
}
