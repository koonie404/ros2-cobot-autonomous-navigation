#pragma once

#include <Arduino.h>

#define PWM_FREQ 20000

typedef struct {
  int M1;
  int M2;
} DCMotorPWM;

void DCMotorPWM_control(DCMotorPWM * motor, int value1, int value2);
void DCMotorPWM_begin(DCMotorPWM * motor);
void DCMotorPWM_setSpeed(DCMotorPWM * motor,int speed);
void DCMotorPWM_stop(DCMotorPWM * motor);
void DCMotorPWM_brake(DCMotorPWM * motor);