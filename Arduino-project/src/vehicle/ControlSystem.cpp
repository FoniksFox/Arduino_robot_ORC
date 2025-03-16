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
    velocity1LastError = 0;
    velocity2LastError = 0;
    velocity1Integral = 0;
    velocity2Integral = 0;
    velocity1Derivative = 0;
    velocity2Derivative = 0;

    velocityKp = 0.5;
    velocityKi = 0.1;
    velocityKd = 0.1;
    velocityIntegralLimit = 100;
    velocityDerivativeLimit = 100;

    lastTime = millis();
}

std::vector<int> ControlSystem::update(double velocity1, double velocity2, double desiredRadius, double desiredVelocity) {
    std::vector<int> controlSignal = {0, 0};

    if (lastTime == 0) lastTime = millis();
    double deltaT = millis() - lastTime + 1;  // Avoid division by zero

    // Reference velocities
    if (desiredRadius == 0) desiredRadius = 1e-6;
    double referenceVelocity1 = desiredVelocity * (1 + WHEEL_DISTANCE / (2 * desiredRadius));
    double referenceVelocity2 = desiredVelocity * (1 - WHEEL_DISTANCE / (2 * desiredRadius));
    Serial.println("Reference Velocity 1: " + String(referenceVelocity1) + ", Reference Velocity 2: " + String(referenceVelocity2));

    // Errors
    double velocity1Error = referenceVelocity1 - velocity1;
    double velocity2Error = referenceVelocity2 - velocity2;
    Serial.println("Velocity 1 Error: " + String(velocity1Error) + ", Velocity 2 Error: " + String(velocity2Error));

    // Compute alpha for rotation priority
    double alpha =0;// std::abs(velocity2Error - velocity1Error) / (std::abs(velocity2Error) + std::abs(velocity1Error) + 1e-6);
    Serial.println("Alpha: " + String(alpha));

    // Integral terms (clamped)
    velocity1Integral = velocity1Integral + velocity1Error * deltaT;
    velocity2Integral = velocity2Integral + velocity2Error * deltaT;
    if (velocity1Integral > velocityIntegralLimit) velocity1Integral = velocityIntegralLimit;
    if (velocity1Integral < -velocityIntegralLimit) velocity1Integral = -velocityIntegralLimit;
    if (velocity2Integral > velocityIntegralLimit) velocity2Integral = velocityIntegralLimit;
    if (velocity2Integral < -velocityIntegralLimit) velocity2Integral = -velocityIntegralLimit;
    Serial.println("Velocity 1 Integral: " + String(velocity1Integral) + ", Velocity 2 Integral: " + String(velocity2Integral));

    // Derivative terms (clamped)
    Serial.println("Delta error1: " + String(velocity1Error - velocity1LastError) + ", Delta error2: " + String(velocity2Error - velocity2LastError));
    velocity1Derivative = (velocity1Error - velocity1LastError) / (deltaT/100);
    velocity2Derivative = (velocity2Error - velocity2LastError) / (deltaT/100);
    if (velocity1Derivative > velocityDerivativeLimit) velocity1Derivative = velocityDerivativeLimit;
    if (velocity1Derivative < -velocityDerivativeLimit) velocity1Derivative = -velocityDerivativeLimit;
    if (velocity2Derivative > velocityDerivativeLimit) velocity2Derivative = velocityDerivativeLimit;
    if (velocity2Derivative < -velocityDerivativeLimit) velocity2Derivative = -velocityDerivativeLimit;
    Serial.println("Velocity 1 Derivative: " + String(velocity1Derivative) + ", Velocity 2 Derivative: " + String(velocity2Derivative));

    // PID Output
    double rawControl1 = velocityKp * velocity1Error + velocityKi * velocity1Integral + velocityKd * velocity1Derivative;
    double rawControl2 = velocityKp * velocity2Error + velocityKi * velocity2Integral + velocityKd * velocity2Derivative;
    Serial.println("Raw Control 1: " + String(rawControl1) + ", Raw Control 2: " + String(rawControl2));

    // Apply rotation weight (alpha)
    controlSignal[0] = rawControl1 * (1 - alpha);
    controlSignal[1] = rawControl2 * (1 + alpha);
    Serial.println("Control Signal 1: " + String(controlSignal[0]) + ", Control Signal 2: " + String(controlSignal[1]));

    // Constraints to 255
    int maxControl = std::max(std::abs(controlSignal[0]), std::abs(controlSignal[1]));
    if (maxControl > 255 && maxControl > 0) {
        controlSignal[0] = controlSignal[0] * 255 / maxControl;
        controlSignal[1] = controlSignal[1] * 255 / maxControl;
    }

    // Update previous values
    velocity1LastError = velocity1Error;
    velocity2LastError = velocity2Error;
    lastTime = millis();

    return controlSignal;
}
