#pragma once

#include <Arduino.h>

typedef struct {
  int M1;
  int M2;
} DCMotor;

void DCMotor_control(DCMotor *dcMotor, int value1, int value2);
void DCMotor_begin(DCMotor *dcMotor);
void DCMotor_forward(DCMotor *dcMotor);
void DCMotor_reverse(DCMotor *dcMotor);
void DCMotor_stop(DCMotor *dcMotor);
void DCMotor_brake(DCMotor *dcMotor);