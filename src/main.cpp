#ifndef UNIT_TEST // disable program main loop while unit testing in progress

#include "dcmotors.h"
#include <Arduino.h>
#include "NewPing.h"
#include "PID_v1.h"

uint8_t pwmPins[] = {6, 9};
uint8_t dirPins[] = {7, 8};
DCMotors motors(pwmPins, dirPins); //PWM, DIR

#define TRIGGER_PIN1  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  2   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     3   // Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 12 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

const unsigned int min_echo = 100;
const unsigned int max_echo = 700;
const unsigned int center = 440;



void setup() {
  Serial.begin(115200);
  myPID.SetOutputLimits(-255.0, 255.0);
}

void emergencyStop() {
    motors.stop();
    myPID.SetMode(MANUAL);
    //Serial.println("STOP");
}

float echo1, echo2;

void loop() {
    echo1 = sonar1.ping_median(3);
    echo2 = sonar2.ping_median(3);

    if (echo1 == 0 || echo2 == 0 || echo1 < min_echo || echo2 < min_echo) {
        emergencyStop();
        return;
    }

    myPID.SetMode(AUTOMATIC);

    Input = echo1 - echo2;
    Serial.print(" ECHOS:");
    Serial.print(echo1);
    Serial.print(",");
    Serial.println(echo2);

    myPID.Compute();

    motors.setPWM(Output);
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
