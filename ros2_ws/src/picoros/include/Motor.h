#pragma once
#include "Encoder.h"
#include "Robot.h"
class Motor {
  private:
  RobotState state;
  double kp;
  double ki;
  double kd;
  int enable;
  int I1;
  int I2;
  double dt; // Change in time (controller period)
  double targetVelocity;
  public:
  Encoder encoder;
  Motor(int enablePin, int I1Pin, int I2Pin, int encoderA, int encoderB);
  double controllerPeriod();
  void setPoint(double targetVel);
  void run(int signedPWM);
  void setGains(double kp, double ki, double kd);
  void activate();
  bool isActive();
};
