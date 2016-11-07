#ifndef UNIT_TEST // disable program main loop while unit testing in progress

#include "dcmotor.h"
#include <Arduino.h>

DCMotor lMotor(6, 7); //PWM, DIR
DCMotor rMotor(9, 8); //PWM, DIR

void setup() {
  Serial.begin(9600);
}

void loop() {
    Serial.println("Both motor stopped");
    lMotor.stop();
    rMotor.stop();
    delay(1000);

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
}

#endif
