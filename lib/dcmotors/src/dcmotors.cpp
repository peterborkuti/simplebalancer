#include "dcmotors.h"

#include <Arduino.h>

void DCMotors::setPWM(const int pwm) {
    for (uint8_t i = 0; i < 2; i++) {
        motor[i].setPWM(pwm);
    }
}

void DCMotors::stop() {
    for (uint8_t i = 0; i < 2; i++) {
        motor[i].setPWM(0);
    }
}

DCMotors::DCMotors(const uint8_t pwmPins[], const uint8_t dirPins[]) {
    //motors = {DCMotor(pwmPins[0], dirPins[0]), DCMotor(pwmPins[1], dirPins[1])};
    for (uint8_t i = 0; i < 2; i++) {
        motor[i] = DCMotor(pwmPins[i], dirPins[i]);
    }
}
