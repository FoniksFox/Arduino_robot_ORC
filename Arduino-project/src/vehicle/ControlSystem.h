#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "interfaces/InterfaceControlSystem.h"
#include <vector>

class ControlSystem : public IControlSystem {
    public:
        void init() override;
        std::vector<double> update(std::vector<double> currentState, std::vector<double> desiredState) override;

    private:
        std::vector<double> kp = {1.0, 1.0, 1.0};
        std::vector<double> ki = {0.1, 0.1, 0.1};
        std::vector<double> kd = {0.01, 0.01, 0.01};

        std::vector<double> error = {0, 0, 0};
        std::vector<double> lastError = {0, 0, 0};
        std::vector<double> integral = {0, 0, 0};
        std::vector<double> derivative = {0, 0, 0};

        // Cross-coupling gains
        std::vector<std::vector<double>> crossKp = {{0.0, 0.1, 0.1}, {0.1, 0.0, 0.1}, {0.1, 0.1, 0.0}};
        std::vector<std::vector<double>> crossKi = {{0.0, 0.01, 0.01}, {0.01, 0.0, 0.01}, {0.01, 0.01, 0.0}};
        std::vector<std::vector<double>> crossKd = {{0.0, 0.001, 0.001}, {0.001, 0.0, 0.001}, {0.001, 0.001, 0.0}};
};

#endif // CONTROL_SYSTEM_H