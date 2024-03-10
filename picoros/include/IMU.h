#pragma once
class IMU {
  private:
  int scl;
  int sda;
  public:
  IMU(int sclPin, int sdaPin);
  void calibrate();
  void acceleration(double& x, double& y, double& z);
  void angularRate(double& x, double& y, double& z);
};
