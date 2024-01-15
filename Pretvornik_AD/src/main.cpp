#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  //Serial.println(analogRead(A0)*5.0/1024.0);
  uint32_t stevilka = analogRead(A0);
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  delay(stevilka);
  digitalWrite(13, LOW);
  digitalWrite(12, HIGH);
  delay(stevilka);
  Serial.println(1000.0/(stevilka * 2.0));
}
