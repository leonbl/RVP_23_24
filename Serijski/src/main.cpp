#include <Arduino.h>
void setup() {
  Serial.begin(9600);
}

void loop() {
  long long int rezultat;
  uint8_t rezu;
  rezu = sizeof(rezultat);
  Serial.println(rezu);
  delay(500);
}

