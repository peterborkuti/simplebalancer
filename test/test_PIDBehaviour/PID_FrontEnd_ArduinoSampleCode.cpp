/******************************************************************
 * PID Simple Example (Augmented with Processing.org Communication)
 * Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 ******************************************************************/
void SerialReceive();
void SerialSend();

#include "dcmotors.h"
#include <Arduino.h>
#include <NewPing.h>
#include <PID_v1.h>

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
double Setpoint = 0, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1.0, Ki=0.0, Kd=0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const unsigned int min_echo = 100;
const unsigned int max_echo = 700;
const unsigned int center = 440;

void setup() {
  Serial.begin(9600);
  Serial.println(13);
  Setpoint = 0;
  myPID.SetOutputLimits(0, 512);
}

void emergencyStop() {
    motors.stop();
    myPID.SetMode(MANUAL);
    Input = 0;
    Output = 0;
    Setpoint = 0;
    myPID.Compute();
    Serial.println("STOP");
}

float echo1, echo2;
unsigned long serialTime; //this will help us know when to talk with processing

void loop() {
    echo1 = sonar1.ping_median(3);
    echo2 = sonar2.ping_median(3);

    if (echo1 == 0 || echo2 == 0 || echo1 < min_echo || echo2 < min_echo) {
        emergencyStop();
        return;
    }

    myPID.SetMode(AUTOMATIC);

    Input = (round(echo1/20.0) - round(echo2/20.0)) * 20.0;

    myPID.Compute();

    if(millis()>serialTime)
    {
      SerialReceive();
      SerialSend();
      serialTime+=500;
    }

    //motors.setPWM(Output);
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  }

  // if the information we got was in the correct format,
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
                                          //   value of "Input"  in most cases (as
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   output blip, then the controller will
    }                                     //   overwrite.

    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //

    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //

    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print(Input);
  Serial.print(" ");
  Serial.print(Output);
  Serial.print(" ");
  Serial.print(myPID.GetKp());
  Serial.print(" ");
  Serial.print(myPID.GetKi());
  Serial.print(" ");
  Serial.print(myPID.GetKd());
  Serial.print(" ");
  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");
  Serial.print(" ");
  if(myPID.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}
