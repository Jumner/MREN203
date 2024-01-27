#pragma once
class ProximitySensor{
  private:
  int pin;
  public:
  ProximitySensor(int analogPin);
  double distance();
};
