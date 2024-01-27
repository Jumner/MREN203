#pragma once
class Encoder{
  private:
  int a;
  int b;
  double currentVelocity;
  public:
  Encoder(int aPin, int bPin);
  double velocity();
  void aInterrupt();
  void bInterrupt();
};
