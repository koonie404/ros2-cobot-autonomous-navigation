#pragma once

#include <DCMotorPWM.h>
#include <Encoder.h>
#include "PID.h"

typedef struct {
    // ── 하드웨어 ──────────────────────────────────────────────────────────────
    DCMotorPWM  motor;
    Encoder     encoder;
    PID         pid;

    // ── 엔코더 파라미터 ───────────────────────────────────────────────────────
    int         PPR;
    int         GRR;
    int         ENC_MAX_POS;
    long        encPosPrev;
    double      filteredRPM;

    // ── FF 파라미터 ───────────────────────────────────────────────────────────
    int         MAX_RPM;
    int         DEAD_ZONE_PWM;
    int         MAX_PWM;

    // ── Speed Ramp 파라미터 ───────────────────────────────────────────────────
    double      accelStep_s;
    double      decelStep_s;

    // ── 상태 ─────────────────────────────────────────────────────────────────
    double      targetRPM;     // 사용자 입력 목표값
    double      setRPM;        // ramp 적용된 현재 명령값
    double      prevSetRPM;
    double      currentRPM;
    int         outputPWM;
    double      offsetPWM;
    int         reversing_cnt;
} MotorControl;

void MotorControl_init(MotorControl *mc,
                       int motorPin1, int motorPin2,
                       int encPin1,   int encPin2,
                       int PPR,       int GRR,
                       int MAX_RPM,   int DEAD_ZONE_PWM, int MAX_PWM,
                       double accelStep_s, double decelStep_s,
                       double kp, double ki, double kd,
                       double alpha);

void MotorControl_begin(MotorControl *mc);
void MotorControl_speedRamp(MotorControl *mc, double dt);
void MotorControl_calculatePWM(MotorControl *mc, double dt);
void MotorControl_applyOutput(MotorControl *mc);
void MotorControl_setTargetRPM(MotorControl *mc, double rpm);
long MotorControl_getEncoderPos(MotorControl *mc);
double MotorControl_getCurrentRPM(MotorControl *mc);
