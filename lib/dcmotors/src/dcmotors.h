/**
 * For controlling two DC motors at the same time
 *
 * Peter Borkuti
 */


#ifndef DCMOTORS_H
#define DCMOTORS_H

#include "dcmotor.h"
#include <Arduino.h>

/**
 * DC Motor control class
 *
 */
class DCMotors {
private:
public:
  DCMotor motor[2];
  DCMotors(const uint8_t pwmPins[], const uint8_t dirPins[]);
  /**
   * Sets the pwm for the motor.
   * @param pwm positiv values for FORWARD, negative for BACKWARD
   */
  void setPWM(const int pwm);
  void stop();
};

#endif
