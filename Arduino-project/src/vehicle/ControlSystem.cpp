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
    // does nothing
}

std::vector<int> ControlSystem::update(double velocity1, double velocity2, int theoreticalVelocity1, int theoreticalVelocity2, double position, double distance) {
    std::vector<int> controlSignal = {0, 0};

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
        
    } else { // Normal operation
        double error = position;
        positionProportionalError = positionKp * (error);
        positionIntegral += error * (millis() - lastTime);
        // Prevent integral windup
        if (positionIntegral > INTEGRAL_LIMIT) positionIntegral = INTEGRAL_LIMIT;
        if (positionIntegral < -INTEGRAL_LIMIT) positionIntegral = -INTEGRAL_LIMIT;

        positionDerivative = positionKd * (error - positionError) / (millis() - lastTime);

        positionError = error;
    }
    double positionControl = positionProportionalError + positionIntegral + positionDerivative;

    // Take distante into account
    double distanceControl = 0;
    distanceControl += distanceKp * (1 / (distance+1));
    distanceControl += distanceKd * (distance - lastDistance) / (millis() - lastTime);
    lastDistance = distance;

    // Calculate velocity errors (TODO: use correctly the integral part)
    double velocityControl1 = 0;
    velocityControl1 = velocityKp1 * (theoreticalVelocity1 - velocity1);
    velocityControl1 += velocityKi1 * (theoreticalVelocity1 - velocity1) * (millis() - lastTime);
    velocityControl1 += velocityKd1 * (theoreticalVelocity1 - velocity1 - velocityError1) / (millis() - lastTime);
    velocityError1 = theoreticalVelocity1 - velocity1;
    if (velocityControl1 > 255) velocityControl1 = 255;
    if (velocityControl1 < -255) velocityControl1 = -255;

    double velocityControl2 = 0;
    velocityControl2 = velocityKp2 * (theoreticalVelocity2 - velocity2);
    velocityControl2 += velocityKi2 * (theoreticalVelocity2 - velocity2) * (millis() - lastTime);
    velocityControl2 += velocityKd2 * (theoreticalVelocity2 - velocity2 - velocityError2) / (millis() - lastTime);
    velocityError2 = theoreticalVelocity2 - velocity2;
    if (velocityControl2 > 255) velocityControl2 = 255;
    if (velocityControl2 < -255) velocityControl2 = -255;

    // Calculate control signal
    controlSignal[0] = velocityControl1 + positionControl + distanceControl;
    controlSignal[1] = velocityControl2 - positionControl + distanceControl;

    lastTime = millis();
    return controlSignal;
}