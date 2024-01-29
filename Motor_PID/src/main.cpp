#include <Arduino.h>
#include "IFX007T-Motor-Control.h"

#define encA 2
#define encB 4

uint8_t speed = 50;
bool direction = 0;
uint8_t state = 0;
uint8_t old_state = 0;
int32_t counter = 0;

// Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void encoder();

void setup()
{
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(encA), encoder, RISING);
  MyMotor.begin();
  MyMotor.setBiDirMotorSpeed(direction, speed);
}

void loop()
{
  Serial.println(counter);
  delay(10);
}

void encoder()
{
  if(digitalRead(encB))
    counter++;
  else
    counter--;
}