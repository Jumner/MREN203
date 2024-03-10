#include <Arduino.h>
#include "Encoder.h"

Encoder::Encoder(int aPin, int bPin) {
  a = aPin;
  b = bPin;
  ticks = 0;
  currentVelocity = 0;
  pinMode(a,INPUT);
  pinMode(b,INPUT);
}
double Encoder::calculateVelocity() {
  /*
  currentVelocity = 2.0 * PI * (double)ticks / (3000.0 * (t_now - t_last))
  */
  // return currentVelocity; // TODO
  return 4.2;
}
void Encoder::aInterrupt() {
  if(digitalRead(b) == LOW) {
    ticks ++;
  } else {
    ticks --;
  }
}
