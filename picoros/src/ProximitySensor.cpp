#include <Arduino.h>
#include "ProximitySensor.h"


ProximitySensor::ProximitySensor(int analogPin){
  pin = analogPin;
  pinMode(pin,INPUT);
}

double ProximitySensor::distance() {
  double reading = analogRead(pin);
  return 6.9; // TODO convert to m

}
