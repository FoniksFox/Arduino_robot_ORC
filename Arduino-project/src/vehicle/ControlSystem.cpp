#include "ControlSystem.h"
#include <vector>
#include <Arduino.h>

// Define static member variables
double ControlSystem::positionError;
double ControlSystem::positionProportionalError;
double ControlSystem::positionIntegral;
double ControlSystem::positionDerivative;
double ControlSystem::positionKp;
double ControlSystem::positionKi;
double ControlSystem::positionKd;

double ControlSystem::distanceError;
double ControlSystem::distanceKp;
double ControlSystem::distanceKd;
double ControlSystem::lastDistance;

double ControlSystem::Kvelocity;
double ControlSystem::Kposition;
double ControlSystem::Kdistance;

double ControlSystem::INTEGRAL_LIMIT;
long long ControlSystem::lastTime;

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
    positionError = 0;
    positionProportionalError = 0;
    positionIntegral = 0;
    positionDerivative = 0;
    positionKp = 0.6;
    positionKi = 0.05;
    positionKd = 0.2;

    distanceError = 0;
    distanceKp = 0.1;
    distanceKd = 0.1;
    lastDistance = 0;

    Kvelocity = 0.4;
    Kposition = 0.7;
    Kdistance = 0.3;

    INTEGRAL_LIMIT = 1000;
    lastTime = millis();
}

std::vector<int> ControlSystem::update(double velocity1, double velocity2, double position, double distance, double desiredVelocity) {
    std::vector<int> controlSignal = {0, 0};

    Serial.print("Inputs - V1: "); Serial.print(velocity1);
    Serial.print(" V2: "); Serial.print(velocity2);
    Serial.print(" Pos: "); Serial.print(position);
    Serial.print(" Dist: "); Serial.print(distance);
    Serial.print(" DesV: "); Serial.println(desiredVelocity);

    if (lastTime == 0)
        lastTime = millis();
    double deltaT = millis() - lastTime + 1e-6;

    // Calculate position errors
    if (position == -1000)
    { // No line detected
        Serial.println("Debug: No line - Turning");
        if (positionError < 0)
        {
            // Turn left
            Serial.println("Debug: Turning Left");
            return {230, -230};
        }
        else
        {
            // Turn right
            Serial.println("Debug: Turning Right");
            return {-230, 230};
        }
    }
    else if (position == 1000)
    { // Intersection detected
        Serial.println("Debug: Intersection Detected");
        positionError = 0;
    }
    else
    { // Normal operation
        double error = position;
        positionProportionalError = positionKp * (error);
        positionIntegral += positionKi * error * deltaT / 1000;
        // Prevent integral windup
        if (positionIntegral > INTEGRAL_LIMIT)
            positionIntegral = INTEGRAL_LIMIT;
        if (positionIntegral < -INTEGRAL_LIMIT)
            positionIntegral = -INTEGRAL_LIMIT;

        positionDerivative = positionKd * (error - positionError) / deltaT;

        positionError = error;
    }
    Serial.println("Position: " + String(position) + ", Error: " + String(positionError) + ", Integral: " + String(positionIntegral) + ", Derivative: " + String(positionDerivative));
    double positionControl = positionProportionalError + positionIntegral + positionDerivative;
    Serial.println("Position Control: " + String(positionControl));

    // Take distante into account
    double distanceControl = 0;
    distanceControl += distanceKp * distance;
    distanceControl += distanceKd * (distance - lastDistance) / deltaT;
    if (distanceControl < 0)
        distanceControl = 0;
    lastDistance = distance;

    // Calculate control signal
    controlSignal[0] = int(velocity1 * Kvelocity + positionControl * Kposition - distanceControl * Kdistance);
    controlSignal[1] = int(velocity2 * Kvelocity - positionControl * Kposition - distanceControl * Kdistance);
    Serial.println("Control Signal: " + String(controlSignal[0]) + ", " + String(controlSignal[1]));

    // Normalize control signal, proportionally to desired velocity
    double maxControlSignal = max(abs(controlSignal[0]), abs(controlSignal[1]));
    Serial.println("Max Control Signal: " + String(maxControlSignal));
    if (controlSignal[0] == maxControlSignal) controlSignal[1] += maxControlSignal / 5;
    if (controlSignal[1] == maxControlSignal) controlSignal[0] += maxControlSignal / 5;
    if (maxControlSignal != 0) {
        controlSignal[0] = controlSignal[0] * desiredVelocity / maxControlSignal;
        controlSignal[1] = controlSignal[1] * desiredVelocity / maxControlSignal;
    } else {
        controlSignal = {int(desiredVelocity), int(desiredVelocity)};
    }

    Serial.print("Final Signals (L,R): "); 
    Serial.print(controlSignal[0]); Serial.print(","); Serial.println(controlSignal[1]);

    lastTime = millis();
    return controlSignal;
}