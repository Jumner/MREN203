#include <Arduino.h>
#include "IMU.h"

IMU::IMU(int sclPin, int sdaPin) {
  scl = sclPin;
  sda = sdaPin;
  // I2C Setup TODO
}

void IMU::calibrate() {
  // Calibration routine TODO
}

void IMU::acceleration(double& x, double& y, double& z) {
  x = 1; // TODO
  y = 2;
  z = 3;
}
void IMU::angularRate(double& x, double& y, double& z) {
  x = 1; // TODO
  y = 2;
  z = 3;
}
