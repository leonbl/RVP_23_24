#include <Arduino.h>
#include "IFX007T-Motor-Control.h"

#define encA 2
#define encB 4

uint8_t speed = 50;
bool direction = 0;
uint8_t state = 0;
uint8_t old_state = 0;
int32_t counter = 0;
uint32_t period = 100;

// PID
float setpoint = 100, e=0, e_prev = 0, e_sum = 0;
float Kp = 1, Kd = 0, Ki = 0;
float KP = Kp;
float KD = Kd / (float)period;
float KI = Ki * (float)period;
float u = 0, up = 0, ui = 0, ud = 0;


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
  uint32_t curTime = millis();
  while(millis() < (curTime + period));
  Serial.println(millis());
  // Izračun PID
  e = setpoint - counter;
  up = KP * e;
  e_sum += e;
  ui = KI * e_sum;
  ud = KD * (e - e_prev);
  u = up + ui + ud;
  e_prev = e;
  MyMotor.setBiDirMotorSpeed(direction, u);
}

void encoder()
{
  if(digitalRead(encB))
    counter++;
  else
    counter--;
}