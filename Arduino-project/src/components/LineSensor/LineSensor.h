#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "../interfaces/InterfaceLineSensor.h"

class LineSensor : public ILineSensor {
    public:
        LineSensor(int IR, int sensors[8]);
        void init() override;
        int getLinePosition() override;

    private:
        int IR;
        int sensors[8];
};

#endif // LINESENSOR_H