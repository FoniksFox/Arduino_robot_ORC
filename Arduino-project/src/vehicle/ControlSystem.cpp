#include "ControlSystem.h"
#include <vector>
#include <algorithm>
#include <Arduino.h>

// Define static member variables
long long ControlSystem::lastTime;
int ControlSystem::previousControlSignal1;
int ControlSystem::previousControlSignal2;

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


void ControlSystem::init() {
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
    double deltaT = static_cast<double>(millis() - lastTime) / 1000.0;  // Convert ms to seconds
    if (deltaT <= 0) deltaT = 1e-6;  // Prevent division by zero

    // Reference velocities
    if (desiredRadius == 0) desiredRadius = 1e-6;
    double referenceVelocity1 = desiredVelocity * (1 + WHEEL_DISTANCE / (2 * desiredRadius));
    double referenceVelocity2 = desiredVelocity * (1 - WHEEL_DISTANCE / (2 * desiredRadius));

    // Errors
    double velocity1Error = referenceVelocity1 - velocity1;
    double velocity2Error = referenceVelocity2 - velocity2;

    // Error constraints (to prevent extreme values)
    const double MAX_ERROR = 200;
    if (velocity1Error > MAX_ERROR) velocity1Error = MAX_ERROR;
    if (velocity1Error < -MAX_ERROR) velocity1Error = -MAX_ERROR;
    if (velocity2Error > MAX_ERROR) velocity2Error = MAX_ERROR;
    if (velocity2Error < -MAX_ERROR) velocity2Error = -MAX_ERROR;

    // Integral terms (clamped)
    velocity1Integral += velocity1Error * deltaT;
    velocity2Integral += velocity2Error * deltaT;
    if (velocity1Integral > velocityIntegralLimit) velocity1Integral = velocityIntegralLimit;
    if (velocity1Integral < -velocityIntegralLimit) velocity1Integral = -velocityIntegralLimit;
    if (velocity2Integral > velocityIntegralLimit) velocity2Integral = velocityIntegralLimit;
    if (velocity2Integral < -velocityIntegralLimit) velocity2Integral = -velocityIntegralLimit;

    // Reset integral if output is saturated
    if (std::abs(previousControlSignal1) >= 220) velocity1Integral -= velocity1Error * deltaT;
    if (std::abs(previousControlSignal2) >= 220) velocity2Integral -= velocity2Error * deltaT;

    // Derivative terms with smoothing
    const double derivativeSmoothing = 0.5;
    double newVelocity1Derivative = (velocity1Error - velocity1LastError) / deltaT;
    double newVelocity2Derivative = (velocity2Error - velocity2LastError) / deltaT;
    velocity1Derivative = derivativeSmoothing * velocity1Derivative + (1 - derivativeSmoothing) * newVelocity1Derivative;
    velocity2Derivative = derivativeSmoothing * velocity2Derivative + (1 - derivativeSmoothing) * newVelocity2Derivative;
    if (velocity1Derivative > velocityDerivativeLimit) velocity1Derivative = velocityDerivativeLimit;
    if (velocity1Derivative < -velocityDerivativeLimit) velocity1Derivative = -velocityDerivativeLimit;
    if (velocity2Derivative > velocityDerivativeLimit) velocity2Derivative = velocityDerivativeLimit;
    if (velocity2Derivative < -velocityDerivativeLimit) velocity2Derivative = -velocityDerivativeLimit;

    // PID Output
    double rawControl1 = velocityKp * velocity1Error + velocityKi * velocity1Integral + velocityKd * velocity1Derivative;
    double rawControl2 = velocityKp * velocity2Error + velocityKi * velocity2Integral + velocityKd * velocity2Derivative;

    // Velocity feedforward
    double feedforward1 = 0.9 * previousControlSignal1;
    double feedforward2 = 0.9 * previousControlSignal2;
    controlSignal[0] += feedforward1;
    controlSignal[1] += feedforward2;

    // Rate limiter (prevent sudden jumps)
    int MAX_CHANGE = deltaT*2000;  // Max change per cycle
    if (MAX_CHANGE > 70) MAX_CHANGE = 70;
    if (controlSignal[0] - previousControlSignal1 > MAX_CHANGE) controlSignal[0] = previousControlSignal1 + MAX_CHANGE;
    if (controlSignal[0] - previousControlSignal1 < -MAX_CHANGE) controlSignal[0] = previousControlSignal1 - MAX_CHANGE;
    if (controlSignal[1] - previousControlSignal2 > MAX_CHANGE) controlSignal[1] = previousControlSignal2 + MAX_CHANGE;
    if (controlSignal[1] - previousControlSignal2 < -MAX_CHANGE) controlSignal[1] = previousControlSignal2 - MAX_CHANGE;

    // Constraints to 255
    int maxControl = std::max(std::abs(controlSignal[0]), std::abs(controlSignal[1]));
    if (maxControl > 255 && maxControl > 0) {
        controlSignal[0] = controlSignal[0] * 255 / maxControl;
        controlSignal[1] = controlSignal[1] * 255 / maxControl;
    }

    // Update previous values
    velocity1LastError = velocity1Error;
    velocity2LastError = velocity2Error;
    previousControlSignal1 = controlSignal[0];
    previousControlSignal2 = controlSignal[1];
    lastTime = millis();

    return controlSignal;
}

