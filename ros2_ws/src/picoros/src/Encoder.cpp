#include <Arduino.h>
#include "Encoder.h"

Encoder::Encoder(int aPin, int bPin) {
  a = aPin;
  b = bPin;
  pinMode(a,INPUT);
  pinMode(b,INPUT);
}
double Encoder::velocity() {
  // return currentVelocity; // TODO
  return 4.2;
}
void Encoder::aInterrupt() {
  // TODO
}
void Encoder::bInterrupt() {
  // TODO
}
