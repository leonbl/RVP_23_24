#include <Arduino.h>
#define tipka 2
#define led1 13
#define led2 12

uint8_t staro_stanje;
uint8_t preklopi = 0;
uint8_t stevec = 0;

void setup()
{
  pinMode(tipka, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  uint8_t stanje_tipke;
  stanje_tipke = digitalRead(tipka);
  if ((staro_stanje == 0) && (stanje_tipke == 1))
  {
    preklopi = 1;
    stevec = stevec + 1;
  }
  else
  {
    preklopi = 0;
  }
  if (preklopi == 1)
  {
    digitalWrite(led1, stevec&0x01);
    digitalWrite(led2, ((stevec>>1)&0x01));
  }
  Serial.println(stanje_tipke);
  staro_stanje = stanje_tipke;
  delay(100);
}
