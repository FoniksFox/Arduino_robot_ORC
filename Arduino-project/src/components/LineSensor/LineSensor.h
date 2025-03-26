#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "../interfaces/InterfaceLineSensor.h"
#include <vector>

class LineSensor : public ILineSensor {
    private:
        int IR;
        int sensors[8];
    public:
        LineSensor(int IR, int sensors[8]);
        void init();
        std::vector<int> readSensors();
        double getLinePosition();
        bool isLineDetected();
        bool isIntersection();
};

#endif // LINESENSOR_H