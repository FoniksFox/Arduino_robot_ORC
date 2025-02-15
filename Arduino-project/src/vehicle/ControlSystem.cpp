#include "ControlSystem.h"
#include <vector>


/* Control System diagram (may need improvements, like kalman filter and maybe a manual bypass):
+-----------------+          ___________           +-----------------+       +-----------------+
|   Desired       |         /           \          |   Controller    |       |   Actuators     |
|    State        |        /             \         |                 |       |                 |
|   (Vector)      +------>|       -       |------->+                 +------>+                 |
|   [v, r, d]     |        \             /         |   PID Control   |       |  Motors, etc.   |
|                 |         \___________/          |   (Vector)      |       |                 |
+-----------------+               ^                +--------+--------+       +-----------------+
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
    // Initialize PID parameters
    kp = {1.0, 1.0, 1.0};
    ki = {0.1, 0.1, 0.1};
    kd = {0.01, 0.01, 0.01};
    error = lastError = integral = derivative = {0, 0, 0};

    // Initialize cross-coupling gains
    crossKp = {{0.0, 0.1, 0.1}, {0.1, 0.0, 0.1}, {0.1, 0.1, 0.0}};
    crossKi = {{0.0, 0.01, 0.01}, {0.01, 0.0, 0.01}, {0.01, 0.01, 0.0}};
    crossKd = {{0.0, 0.001, 0.001}, {0.001, 0.0, 0.001}, {0.001, 0.001, 0.0}};
}

const double INTEGRAL_LIMIT = 10.0;
std::vector<double> ControlSystem::update(std::vector<double> currentState, std::vector<double> desiredState) {
    std::vector<double> controlSignal = {0, 0, 0};
    // Calculate errors
    for (size_t i = 0; i < error.size(); ++i) {
        error[i] = desiredState[i] - currentState[i];
        integral[i] += error[i];
        // Prevent integral windup
        if (integral[i] > INTEGRAL_LIMIT) integral[i] = INTEGRAL_LIMIT;
        if (integral[i] < -INTEGRAL_LIMIT) integral[i] = -INTEGRAL_LIMIT;
        derivative[i] = error[i] - lastError[i];
        controlSignal[i] = kp[i] * error[i] + ki[i] * integral[i] + kd[i] * derivative[i];
        lastError[i] = error[i];
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
    std::vector<double> feedforward = {desiredState[0] * 0.1, desiredState[1] * 0.1, desiredState[2] * 0.1};
    for (size_t i = 0; i < controlSignal.size(); ++i) {
        controlSignal[i] += feedforward[i];  // Adjust for faster response
    }

    return controlSignal;
}