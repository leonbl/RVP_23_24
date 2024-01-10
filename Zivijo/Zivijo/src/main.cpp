#include <Arduino.h>

void setup() {
  // nastavimo pin z imenom 12 kot izhod
  pinMode(12,OUTPUT);
}

void loop() {
  // pin 12 nastavimo na visoki nivo (5V)
  digitalWrite(12,HIGH);
  // ne počni nič 1000ms
  delay(1000);
  // pin 12 nastavimo na nizek nivo (0V)
  digitalWrite(12,LOW);
  delay(1000);
}