/**
 * Simple DC Motor Control Class
 *
 * Peter Borkuti
 */


#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

/**
 * DC Motor control class
 *
 */
class DCMotor {
private:
/**
 * arduino pin for pwm
 */
  uint8_t pwmPin;
/**
 * arduino pin for direction. If it is HIGH, go FORWARD
 * if it is low, go backward
 */
  uint8_t dirPin;
/**
 * stores the actual state of the pwm pin
 */
  int statePWM;

  uint8_t rawPWM;
/**
 * stores the actual state of the direction pin
 */
uint8_t stateDir;
public:
static const uint8_t FORWARD;
static const uint8_t NOTMOVING;
static const uint8_t BACKWARD;
DCMotor() {};
  DCMotor(const uint8_t _pwmPin, const uint8_t _dirPin);
  /**
   * Sets the pwm for the motor.
   * @param pwm positiv values for FORWARD, negative for BACKWARD
   */
  void setPWM(const int _pwm);
  /**
   * gets the set pwm value. It can be negative when motor moves backward
   * @return negative or positive pwm value
   */
  int getPWM() const;
  uint8_t getRawPWM() const;
  uint8_t getDirection() const;
  void stop();
};

#endif
