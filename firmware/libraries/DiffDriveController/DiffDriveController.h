#pragma once

#include <MotorControl.h>
#include <Arduino.h>

typedef struct {
  MotorControl mcLeft;
  MotorControl mcRight;
  double leftTargetRPM;
  double rightTargetRPM;
} DiffDriveController;

void DiffDriveController_begin(DiffDriveController* ddc);
void DiffDriveController_setTargetRPMs(DiffDriveController* ddc, double lRPM, double rRPM);
void DiffDriveController_updateMotorSpeeds(DiffDriveController* ddc, double dt);
void DiffDriveController_getPresentPositions(DiffDriveController* ddc, long* lPP, long* rPP);
void DiffDriveController_getPresentRPMs(DiffDriveController* ddc, double* lRPM, double* rRPM);