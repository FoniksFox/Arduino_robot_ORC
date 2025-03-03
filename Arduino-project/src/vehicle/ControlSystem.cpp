#include "ControlSystem.h"
#include <vector>
#include <Arduino.h>


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

void ControlSystem::init() {
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

std::vector<int> ControlSystem::update(double velocity1, double velocity2, double position, double distance) {
    std::vector<int> controlSignal = {0, 0};
    if (lastTime == 0) lastTime = millis();
    double deltaT = millis() - lastTime + 1e-6;

    // Calculate position errors
    if (position == -10) { // No line detected
        if (positionError < 0) {
            // Turn left
            return {230, -230};
        } else {
            // Turn right
            return {-230, 230};
        }
    } else if (position == 10) { // Intersection detected
        positionError = 0;
    } else { // Normal operation
        double error = position;
        positionProportionalError = positionKp * (error);
        positionIntegral += positionKi * error * deltaT / 1000;
        // Prevent integral windup
        if (positionIntegral > INTEGRAL_LIMIT) positionIntegral = INTEGRAL_LIMIT;
        if (positionIntegral < -INTEGRAL_LIMIT) positionIntegral = -INTEGRAL_LIMIT;

        positionDerivative = positionKd * (error - positionError) / deltaT;

        positionError = error;
    }
    double positionControl = positionProportionalError + positionIntegral + positionDerivative;

    // Take distante into account
    double distanceControl = 0;
    distanceControl += distanceKp * distance;
    distanceControl += distanceKd * (distance - lastDistance) / deltaT;
    if (distanceControl < 0) distanceControl = 0;
    lastDistance = distance;

    // Calculate control signal
    controlSignal[0] = int(velocity1*Kvelocity + positionControl*Kposition - distanceControl*Kdistance);
    controlSignal[1] = int(velocity2*Kvelocity - positionControl*Kposition - distanceControl*Kdistance);

    // Normalize control signal, proportionally, to a max of [-255, 255]
    double maxControlSignal = max(abs(controlSignal[0]), abs(controlSignal[1]));
    if (maxControlSignal > 255) {
        controlSignal[0] = controlSignal[0] * 255 / maxControlSignal;
        controlSignal[1] = controlSignal[1] * 255 / maxControlSignal;
    }

    lastTime = millis();
    return controlSignal;
}