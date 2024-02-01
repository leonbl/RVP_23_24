#include <Arduino.h>
#include "IFX007T-Motor-Control.h"
#include <QuickPID.h>

#define encA 2
#define encB 4

uint8_t speed = 50;
bool direction = 0;
uint8_t state = 0;
uint8_t old_state = 0;
int32_t counter = 0;
uint32_t period = 100;

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

float Kp = 0.2, Ki = 0.5, Kd = 0;

//Specify PID links
QuickPID myPID(&Input, &Output, &Setpoint);

// Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void encoder();

void setup()
{
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(encA), encoder, RISING);
  MyMotor.begin();
  MyMotor.setBiDirMotorSpeed(direction, speed);
  Setpoint = 1000;

  //apply PID gains
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-255, 255);
  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);
}

void loop()
{
  uint32_t curTime = millis();
  while (millis() < (curTime + period))
    ;
  Input = counter;
  myPID.Compute();
  if (Output > 0)
  {
    direction = 0;
  }
  else
  { 
    direction = 1;
  }

  MyMotor.setBiDirMotorSpeed(direction, abs(Output));

  Serial.print("Output: ");
  Serial.print(Output);
  Serial.print(" dir: ");
  Serial.print(direction);
  Serial.print(" count: ");
  Serial.println(counter);
}

void encoder()
{
  if (digitalRead(encB))
    counter++;
  else
    counter--;
}