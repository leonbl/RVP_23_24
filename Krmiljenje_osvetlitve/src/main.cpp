#include <Arduino.h>
#define led1 9
#define led2 10
#define led3 11

int n = 200;
int e = 0;
int setpoint = 400;
float K = 0.5;

void setup()
{
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
    analogWrite(led1, n);
    analogWrite(led2, n);
    analogWrite(led3, n);
    int output = analogRead(A1);
    e = setpoint - output;
    n = e * K;
    n = constrain(n, 0, 255);

    Serial.print(output);
    Serial.print(" ");
    Serial.print(n);
    Serial.print(" ");
    Serial.println(e);

}
