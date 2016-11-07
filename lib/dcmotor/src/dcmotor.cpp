#include "dcmotor.h"

#include <Arduino.h>

const uint8_t DCMotor::FORWARD = 0;
const uint8_t DCMotor::NOTMOVING = 1;
const uint8_t DCMotor::BACKWARD = 2;

void DCMotor::setPWM(int pwm) {
    statePWM = pwm;
    stateDir = (pwm < 0) ? BACKWARD : ((pwm > 0) ? FORWARD : NOTMOVING);

    if (pwm < 0) {
        pwm = -pwm;
    }

    if (pwm > 255) {
        pwm = 255;
    }

    uint8_t rawDir = LOW;

    if (stateDir == FORWARD) {
      rawPWM = map(pwm, 0, 255, 128, 255);
    }
    else if (stateDir == BACKWARD) {
      rawPWM = map(pwm, 0, 255, 128, 0);
      rawDir = HIGH;
    }
    else {
        rawPWM = 0;
    }

    analogWrite(pwmPin, rawPWM);
    digitalWrite(dirPin, rawDir);
}

void DCMotor::stop() {
    setPWM(0);
}

int DCMotor::getPWM() const {
    return statePWM;
}

uint8_t DCMotor::getRawPWM() const {
    return rawPWM;
}

uint8_t DCMotor::getDirection() const {
    return stateDir;
}

DCMotor::DCMotor(uint8_t _pwmPin, uint8_t _dirPin) {
  pwmPin = _pwmPin;
  dirPin = _dirPin;
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  stop();
}
