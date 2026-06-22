#include "DCMotorPWM.h"

void DCMotorPWM_control(DCMotorPWM * motor, int value1, int value2) {
  ledcWrite(motor->M1, value1);
  ledcWrite(motor->M2, value2); 
}

void DCMotorPWM_begin(DCMotorPWM * motor) {
  ledcAttach(motor->M1, PWM_FREQ, 10);
  ledcAttach(motor->M2, PWM_FREQ, 10);
}

void DCMotorPWM_setSpeed(DCMotorPWM * motor,int speed) {
  speed=constrain(speed, -1023, 1023);
  int duty_cycle=abs(speed);  
  if(speed>=0) {
    DCMotorPWM_control(motor, duty_cycle, 0); 
  } else if(speed<0) {
    DCMotorPWM_control(motor, 0,duty_cycle);
  }
}

void DCMotorPWM_stop(DCMotorPWM * motor) {
  DCMotorPWM_control(motor, 0, 0);
}

void DCMotorPWM_brake(DCMotorPWM * motor) {
  DCMotorPWM_control(motor, 1023, 1023);
}