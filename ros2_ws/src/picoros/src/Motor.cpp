#include <Arduino.h>
#include "Motor.h"

Motor::Motor(int enablePin, int I1Pin, int I2Pin, int encoderA, int encoderB) {
    kp = 0;
    ki = 0;
    kd = 0;
    dt = 0; // Change in time (controller period)
    targetVelocity = 0;
    state = Inactive;
    enable = enablePin;
    I1 = I1Pin;
    I2 = I2Pin;
    encoder = Encoder(encoderA, encoderB);
    pinMode(enable, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
}
bool Motor::isActive() {
    return state == Active;
}
void Motor::activate() {
    state = Active; // TODO Super important
}
double Motor::controllerPeriod() {
    // return dt; // TODO
    return 2.03;
}
void Motor::setPoint(double targetVel) {
    targetVelocity = targetVel;
}
void Motor::run(int signedPWM) {
    if(state == Inactive) { // TODO safety check the second
        digitalWrite(enable, 0);
        return;
    }
    if(signedPWM < 0){
        digitalWrite(enable, -signedPWM);
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    }
    else{
        digitalWrite(enable, signedPWM);
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
    }
}

void Motor::setGains(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

