#include "DCMotor.h"

void DCMotor_control(DCMotor *dcMotor, int value1, int value2) {
  digitalWrite(dcMotor->M1, value1);
  digitalWrite(dcMotor->M2, value2);
}

void DCMotor_begin(DCMotor *dcMotor) {
  pinMode(dcMotor->M1,OUTPUT);
  pinMode(dcMotor->M2,OUTPUT);    
}

void DCMotor_forward(DCMotor *dcMotor) {
  DCMotor_control(dcMotor,HIGH,LOW);
}

void DCMotor_reverse(DCMotor *dcMotor) {
  DCMotor_control(dcMotor, LOW,HIGH);
}

void DCMotor_stop(DCMotor *dcMotor) {
  DCMotor_control(dcMotor,LOW,LOW);
}

void DCMotor_brake(DCMotor *dcMotor) {
  DCMotor_control(dcMotor,HIGH,HIGH);
}