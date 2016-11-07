#include "dcmotor.h"
#include <Arduino.h>
#include <unity.h>

#ifdef UNIT_TEST

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
    UNITY_END();
}

void loop() {

}

#endif
