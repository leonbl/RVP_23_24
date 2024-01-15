#include <Arduino.h>

uint32_t de = 10;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  de = de + 10;
  for (char x = 'a'; x <= 'z'; x++)
  {
    Serial.println(x);
    delay(500);
  }

}
