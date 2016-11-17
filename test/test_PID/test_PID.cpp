#include <Arduino.h>
#include <PID_v1.h>
#include <unity.h>

#ifdef UNIT_TEST

void pid_zero(void) {
    double Kp=0, Ki=0, Kd=0;
    double Setpoint = 0, Input = 0, Output = 0;
    PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    for (Input = -100; Input < 100; Input++) {
        pid.Compute();
        TEST_ASSERT_EQUAL(0, Output);
    }

    pid.SetTunings(0, 1, 1);
    for (Input = -100; Input < 100; Input++) {
        pid.Compute();
        TEST_ASSERT_EQUAL(0, Output);
    }


    pid.SetControllerDirection(REVERSE);
    for (Input = -100; Input < 100; Input++) {
        pid.Compute();
        TEST_ASSERT_EQUAL(0, Output);
    }
}

void pid_pOnly(void) {
    double Setpoint = 0, Input = 0, Output = 0;
    double Kp=1, Ki=0, Kd=0;
    PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    pid.SetOutputLimits(-255, 255);
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(1);
    for (Input = -100; Input < 100; Input++) {
        delay(2);
        pid.Compute();
        TEST_ASSERT_EQUAL(-Input, Output);
    }

    pid.SetControllerDirection(REVERSE);
    for (Input = -100; Input < 100; Input++) {
        pid.Compute();
        TEST_ASSERT_EQUAL(-Input, Output);
    }
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(pid_zero);
    RUN_TEST(pid_pOnly);
    UNITY_END();
}

void loop() {

}

#endif
