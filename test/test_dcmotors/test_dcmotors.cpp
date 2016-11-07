#include "dcmotors.h"
#include <Arduino.h>
#include <unity.h>

#ifdef UNIT_TEST

uint8_t pwmPins[] = {6, 9};
uint8_t dirPins[] = {7, 8};
DCMotors motors(pwmPins, dirPins); //PWM, DIR

void motors_stopped(void) {
    TEST_ASSERT_EQUAL(0, motors.motor[0].getPWM());
    TEST_ASSERT_EQUAL(0, motors.motor[0].getRawPWM());
    TEST_ASSERT_EQUAL(0, motors.motor[1].getPWM());
    TEST_ASSERT_EQUAL(0, motors.motor[1].getRawPWM());

    TEST_ASSERT_EQUAL(motors.motor[0].NOTMOVING, motors.motor[0].getDirection());
    TEST_ASSERT_EQUAL(motors.motor[1].NOTMOVING, motors.motor[0].getDirection());
}

void motors_max_forward(void) {
    TEST_ASSERT_EQUAL(255, motors.motor[0].getRawPWM());
    TEST_ASSERT_EQUAL(motors.motor[0].FORWARD, motors.motor[0].getDirection());
    TEST_ASSERT_EQUAL(255, motors.motor[1].getRawPWM());
    TEST_ASSERT_EQUAL(motors.motor[1].FORWARD, motors.motor[1].getDirection());
}

void motors_max_backward(void) {
    TEST_ASSERT_EQUAL(0, motors.motor[0].getRawPWM());
    TEST_ASSERT_EQUAL(motors.motor[0].BACKWARD, motors.motor[0].getDirection());
    TEST_ASSERT_EQUAL(0, motors.motor[1].getRawPWM());
    TEST_ASSERT_EQUAL(motors.motor[1].BACKWARD, motors.motor[1].getDirection());
}


DCMotor motor(6, 5);

void motor_stopped(void) {
    TEST_ASSERT_EQUAL(0, motor.getPWM());
    TEST_ASSERT_EQUAL(0, motor.getRawPWM());
    TEST_ASSERT_EQUAL(motor.NOTMOVING, motor.getDirection());
}

void motor_max_forward(void) {
    TEST_ASSERT_EQUAL(255, motor.getRawPWM());
    TEST_ASSERT_EQUAL(motor.FORWARD, motor.getDirection());
}

void motor_max_backward(void) {
    TEST_ASSERT_EQUAL(0, motor.getRawPWM());
    TEST_ASSERT_EQUAL(motor.BACKWARD, motor.getDirection());
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(motor_stopped);
    motor.setPWM(1000);
    RUN_TEST(motor_max_forward);
    motor.setPWM(-1000);
    RUN_TEST(motor_max_backward);
    motor.stop();
    RUN_TEST(motor_stopped);

    RUN_TEST(motors_stopped);
    motors.setPWM(1000);
    RUN_TEST(motors_max_forward);
    motors.setPWM(-1000);
    RUN_TEST(motors_max_backward);
    motors.stop();
    RUN_TEST(motors_stopped);

    UNITY_END();
}

void loop() {

}

#endif
