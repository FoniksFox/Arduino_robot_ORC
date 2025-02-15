#ifndef IVEHICLE_H
#define IVEHICLE_H

class IVehicle {
    public:
        virtual void init() = 0;
        virtual void update() = 0;
};

#endif // IVEHICLE_H