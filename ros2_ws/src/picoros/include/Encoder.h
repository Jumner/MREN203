#pragma once
class Encoder{
  private:
  int ticks;
  int a;
  int b;
  double lastT;
  public:
  double currentVelocity;
  Encoder(int aPin, int bPin);
  double calculateVelocity();
  void aInterrupt();
};
