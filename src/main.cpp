#ifndef UNIT_TEST // disable program main loop while unit testing in progress

#include "dcmotors.h"
#include <Arduino.h>

uint8_t pwmPins[] = {6, 9};
uint8_t dirPins[] = {7, 8};
DCMotors motors(pwmPins, dirPins); //PWM, DIR

void setup() {
  Serial.begin(9600);
}

void loop() {
    Serial.println("Both motor stopped");
    motors.stop();
    delay(1000);

    Serial.println("Both motor fw");
    motors.setPWM(1000);
    delay(1000);
    motors.stop();

    Serial.println("Both motor bw");
    motors.setPWM(-1000);
    delay(1000);
    motors.stop();
}

void dcmotor_test() {
/*
    Serial.println("Left motor forward");
    lMotor.setPWM(1000);
    delay(1000);
    lMotor.stop();

    Serial.println("Right motor forward");
    rMotor.setPWM(1000);
    delay(1000);
    rMotor.stop();

    Serial.println("Left motor bw");
    lMotor.setPWM(-1000);
    delay(1000);
    lMotor.stop();

    Serial.println("Right motor bw");
    rMotor.setPWM(-1000);
    delay(1000);
    rMotor.stop();

    Serial.println("Both motor fw");
    lMotor.setPWM(1000);
    rMotor.setPWM(1000);
    delay(1000);
    rMotor.stop();
    lMotor.stop();

    Serial.println("Both motor bw");
    lMotor.setPWM(-1000);
    rMotor.setPWM(-1000);
    delay(1000);
    rMotor.stop();
    lMotor.stop();
*/
}

#endif
