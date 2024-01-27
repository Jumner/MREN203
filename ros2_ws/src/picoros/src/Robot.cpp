#include <Arduino.h>
#include "Robot.h"

Robot::Robot(int ea, int eb, int i1, int i2, int i3, int i4, int frontProx, int leftProx, int rightProx, int leftEncoderA, int leftEncoderB, int rightEncoderA, int rightEncoderB, int scl, int sda) {
  state = Inactive;
  leftMotor = Motor(ea, i1, i2, leftEncoderA, leftEncoderB); // TODO double check
  rightMotor = Motor(eb, i4, i3 rightEncoderB, rightEncoderA); // For this one too
  frontProximity = ProximitySensor(frontProx);
  leftProximity = ProximitySensor(leftProx);
  rightProximity = ProximitySensor(rightProx);
  imu = IMU(scl, sda);
}
void Robot::activate(double kp, double ki, double kd) {
  state = Active; // TODO Very important
  leftMotor.setGains(kp, ki, kd);
  rightMotor.setGains(kp, ki, kd);
  leftMotor.activate();
  rightMotor.activate();
}
bool Robot::isActive() {
  return state == Active && leftMotor.isActive() && rightMotor.isActive();
}
void Robot::setMotorVelocities(mren_interfaces__msg__PicoMotorCommands msgin) {
  double leftVel = msgin->left_wheel_velocity;
  double rightVel = msgin->right_wheel_velocity;
  if(isActive()) { // Safety check
    leftMotor.setPoint(leftVel);
    rightMotor.setPoint(rightVel);
  }
}
void Robot::takeReading(mren_interfaces__msg__PicoSensorOutput * msgin) {
  msgin->left_wheel_velocity = leftMotor.encoder.velocity();
  msgin->right_wheel_velocity = rightMotor.encoder.velocity();
  msgin->left_PID_period = leftMotor.controllerPeriod();
  msgin->right_PID_period = rightMotor.controllerPeriod();
  msgin->proximity_front = frontProximity.distance();
  msgin->proximity_left = leftProximity.distance();
  msgin->proximity_right = rightProximity.distance();
  imu.acceleration(msgin->acceleration.ax, msgin->acceleration.ay, msgin->acceleration.az);
  imu.angularRate(msgin->angular_rate.rx, msgin->angular_rate.ry, msgin->angular_rate.rz);
}
