#include "DiffDriveController.h"

void DiffDriveController_begin(DiffDriveController* ddc) {
  MotorControl_init(
    &ddc->mcLeft,
    4, 5,           // motor pins
    6, 7,           // encoder pins
    7, 298,         // PPR, GRR
    60, 580, 980,   // MAX_RPM, DEAD_ZONE_PWM, MAX_PWM
    30.0, 85.7,     // accelStep_s, decelStep_s
    32.0, 32.0, 1.0,// kp, ki, kd
    0.1             // alpha
  );

  MotorControl_init(
    &ddc->mcRight,
    9, 10,           // motor pins
    11,12,           // encoder pins
    7, 298,         // PPR, GRR
    60, 580, 980,   // MAX_RPM, DEAD_ZONE_PWM, MAX_PWM
    30.0, 85.7,     // accelStep_s, decelStep_s
    32.0, 32.0, 1.0,// kp, ki, kd
    0.1             // alpha
  );

  MotorControl_begin(&ddc->mcLeft);
  MotorControl_begin(&ddc->mcRight);
}

void DiffDriveController_setTargetRPMs(DiffDriveController* ddc, double lRPM, double rRPM) {
  MotorControl_setTargetRPM(&ddc->mcLeft, lRPM);
  MotorControl_setTargetRPM(&ddc->mcRight, rRPM);
}

void DiffDriveController_updateMotorSpeeds(DiffDriveController* ddc, double dt) {
  MotorControl_speedRamp(&ddc->mcLeft, dt);
  MotorControl_speedRamp(&ddc->mcRight, dt);

  MotorControl_calculatePWM(&ddc->mcLeft, dt);
  MotorControl_calculatePWM(&ddc->mcRight, dt);

  MotorControl_applyOutput(&ddc->mcLeft);
  MotorControl_applyOutput(&ddc->mcRight);
}

void DiffDriveController_getPresentPositions(DiffDriveController* ddc, long* lPP, long* rPP) {
  *lPP = MotorControl_getEncoderPos(&ddc->mcLeft);
  *rPP = MotorControl_getEncoderPos(&ddc->mcRight);
}

void DiffDriveController_getPresentRPMs(DiffDriveController* ddc, double* lRPM, double* rRPM) {
  *lRPM = MotorControl_getCurrentRPM(&ddc->mcLeft);
  *rRPM = MotorControl_getCurrentRPM(&ddc->mcRight);
}