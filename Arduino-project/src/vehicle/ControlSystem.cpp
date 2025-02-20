#include "ControlSystem.h"
#include <vector>
#include <Arduino.h>


/* Control System diagram (may need improvements, like kalman filter and maybe a manual bypass):
+-----------------+          ___________           +-----------------+       +-----------------+
|   Desired       |         /           \          |   Controller    |       |   Actuators     |
|    State        |      - /             \         |                 |       |                 |
|   (Vector)      +------>|               |------->+                 +------>+                 |
|   [v, r, d]     |        \             /         |   PID Control   |       |  Motors, etc.   |
|                 |         \___________/          |   (Vector)      |       |                 |
+-----------------+               ^ +              +--------+--------+       +-----------------+
                                  |                         |
                                  |                         |
                                  |                         v
                         +--------+--------+       +--------+--------+
                         |   Sensors       |       |   Vehicle       |
                         |   (Vector)      |       |  Dynamics       |
                         |   [v, r, d]     +<------+                 |
                         |                 |       |                 |
                         +-----------------+       +-----------------+
*/

void ControlSystem::init() {
    // does nothing
}

std::vector<double> ControlSystem::update(std::vector<double> currentState, std::vector<double> desiredState) {
    std::vector<double> controlSignal = {0, 0, 0};
    // Calculate errors
    for (size_t i = 0; i < error.size(); ++i) {
        error[i] = desiredState[i] - currentState[i];

        integral[i] += error[i] * (millis() - lastTime);
        // Prevent integral windup
        if (integral[i] > INTEGRAL_LIMIT) integral[i] = INTEGRAL_LIMIT;
        if (integral[i] < -INTEGRAL_LIMIT) integral[i] = -INTEGRAL_LIMIT;

        derivative[i] = error[i] - lastError[i] / (millis() - lastTime);

        controlSignal[i] = kp[i] * error[i] + ki[i] * integral[i] + kd[i] * derivative[i];
    }

    // Apply cross-coupling terms
    for (size_t i = 0; i < controlSignal.size(); ++i) {
        for (size_t j = 0; j < controlSignal.size(); ++j) {
            if (i != j) {
                controlSignal[i] += crossKp[i][j] * error[j] + crossKi[i][j] * integral[j] + crossKd[i][j] * derivative[j];
            }
        }
    }

    // Feedforward control
    std::vector<double> feedforward = {desiredState[0] * FEEDFORWARD_GAIN, desiredState[1] * FEEDFORWARD_GAIN, desiredState[2] * FEEDFORWARD_GAIN};
    for (size_t i = 0; i < controlSignal.size(); ++i) {
        controlSignal[i] += feedforward[i];  // Adjust for faster response
    }

    lastTime = millis();
    lastError = error;

    return controlSignal;
}