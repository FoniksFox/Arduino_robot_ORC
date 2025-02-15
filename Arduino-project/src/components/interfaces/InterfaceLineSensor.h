#ifndef ILINESENSOR_H
#define ILINESENSOR_H

class ILineSensor {
    public:
        virtual void init() = 0;
        virtual int getLinePosition() = 0;
};

#endif // ILINESENSOR_H