#ifndef ILINESENSOR_H
#define ILINESENSOR_H

class ILineSensor {
    public:
        virtual void init() = 0;
        virtual double getLinePosition() = 0;
        virtual bool isLineDetected() = 0;
};

#endif // ILINESENSOR_H