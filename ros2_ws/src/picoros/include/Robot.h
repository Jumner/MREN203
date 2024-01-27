#pragma once
#include "Motor.h"
#include "IMU.h"
#include "ProximitySensor.h"
#include <mren_interfaces/msg/pico_motor_commands.h>
#include <mren_interfaces/msg/pico_sensor_output.h>

enum RobotState {
  Inactive,
  Active
};

class Robot {
  private:
  RobotState state;
  public:
  Motor leftMotor;
  Motor rightMotor;
  ProximitySensor frontProximity;
  ProximitySensor leftProximity;
  ProximitySensor rightProximity;
  IMU imu;
  Robot(int ea, int eb, int i1, int i2, int i3, int i4, int frontProx, int leftProx, int rightProx, int leftEncoderA, int leftEncoderB, int rightEncoderA, int rightEncoderB, int scl, int sda);
  void activate(double kp, double ki, double kd);
  bool isActive();
  void setMotorVelocities(mren_interfaces__msg__PicoMotorCommands msgin);
  void takeReading(mren_interfaces__msg__PicoSensorOutput * msgin);
};
