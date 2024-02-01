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
int32_t old_counter = 0;
uint32_t period = 10;
int32_t pot=0;

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

float Kp = 1, Ki = 2, Kd = 0;

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
  Setpoint = 50;

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
  pot = counter - old_counter;  
  Input = pot;

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
  Serial.print(" pot: ");
  Serial.print(pot);
  Serial.print(" count: ");
  Serial.println(counter);
  old_counter = counter;
}

void encoder()
{
  if (digitalRead(encB))
    counter++;
  else
    counter--;
}