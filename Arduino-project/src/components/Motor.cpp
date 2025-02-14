#include "Motor.h"

Motor::Motor(IMotorController& motorController, int motor) : motorController(motorController), motor(motor) {}

void Motor::init() {
    motorController.init();
}

void Motor::setSpeed(int speed) {
    motorController.setMotorSpeed(motor, speed);
}

void Motor::stop() {
    motorController.stopMotor(motor);
}