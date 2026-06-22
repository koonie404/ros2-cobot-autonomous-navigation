#include "MotorControl.h"
#include <Arduino.h>

// ── 초기화 ────────────────────────────────────────────────────────────────────
void MotorControl_init(MotorControl *mc,
                       int motorPin1, int motorPin2,
                       int encPin1,   int encPin2,
                       int PPR,       int GRR,
                       int MAX_RPM,   int DEAD_ZONE_PWM, int MAX_PWM,
                       double accelStep_s, double decelStep_s,
                       double kp, double ki, double kd,
                       double alpha)
{
    // 하드웨어
    mc->motor   = {motorPin1, motorPin2};
    mc->encoder = {encPin1, encPin2, 0, portMUX_INITIALIZER_UNLOCKED};

    // PID
    PID_init(&mc->pid, kp, ki, kd, -1023, 1023, alpha);

    // 엔코더 파라미터
    mc->PPR         = PPR;
    mc->GRR         = GRR;
    mc->ENC_MAX_POS = PPR * 4 * GRR;
    mc->encPosPrev  = 0;
    mc->filteredRPM = 0;

    // FF 파라미터
    mc->MAX_RPM       = MAX_RPM;
    mc->DEAD_ZONE_PWM = DEAD_ZONE_PWM;
    mc->MAX_PWM       = MAX_PWM;

    // Speed Ramp
    mc->accelStep_s = accelStep_s;
    mc->decelStep_s = decelStep_s;

    // 상태 초기화
    mc->targetRPM     = 0;
    mc->setRPM        = 0;
    mc->prevSetRPM    = 0;
    mc->currentRPM    = 0;
    mc->outputPWM     = 0;
    mc->offsetPWM     = 0;
    mc->reversing_cnt = 0;
}

void MotorControl_begin(MotorControl *mc)
{
    DCMotorPWM_begin(&mc->motor);
    Encoder_begin(&mc->encoder);
}

// ── FeedForward ───────────────────────────────────────────────────────────────
static int _getFeedForwardPWM(MotorControl *mc, double rpm)
{
    if (rpm == 0) return 0;

    double ratio = fabs(rpm) / (double)mc->MAX_RPM;
    ratio = constrain(ratio, 0.0, 1.0);

    int ffPWM = (int)(mc->DEAD_ZONE_PWM
               + (mc->MAX_PWM - mc->DEAD_ZONE_PWM) * ratio);
    return (rpm > 0) ? ffPWM : -ffPWM;
}

// ── Speed Ramp ────────────────────────────────────────────────────────────────
void MotorControl_speedRamp(MotorControl *mc, double dt)
{
    double diff = mc->targetRPM - mc->setRPM;
    if (fabs(diff) < 0.1) {
        mc->setRPM = mc->targetRPM;
        return;
    }

    // 방향 전환 확인
    bool isReversing     = (mc->setRPM * mc->targetRPM < 0);
    bool isAccelerating  = (!isReversing
                           && fabs(mc->targetRPM) > fabs(mc->setRPM));
    double maxStep = isAccelerating
                   ? mc->accelStep_s * dt
                   : mc->decelStep_s * dt;

    if (isReversing) {
        if (mc->setRPM > 0) mc->setRPM = max(mc->setRPM - maxStep, 0.0);
        else                mc->setRPM = min(mc->setRPM + maxStep, 0.0);
    } else {
        if (diff > 0) mc->setRPM += min(diff,        maxStep);
        else          mc->setRPM -= min(fabs(diff),   maxStep);
    }

    if (fabs(mc->setRPM) < 0.05)
        mc->setRPM = 0;
}

// ── PWM 계산 ──────────────────────────────────────────────────────────────────
void MotorControl_calculatePWM(MotorControl *mc, double dt)
{
    // 엔코더 → RPM
    long encPosCurr  = Encoder_getPos(&mc->encoder);
    long dEncPos     = encPosCurr - mc->encPosPrev;
    mc->encPosPrev   = encPosCurr;

    double rawRPM    = (double)dEncPos / mc->ENC_MAX_POS / dt * 60.0;
    mc->filteredRPM  = 0.7 * mc->filteredRPM + 0.3 * rawRPM;
    mc->currentRPM   = mc->filteredRPM;

    // 방향 반전 감지 (speedRamp 안전망)
    if (mc->setRPM * mc->prevSetRPM < 0) {
        PID_reset(&mc->pid);
        mc->outputPWM     = 0;
        mc->prevSetRPM    = 0;
        mc->reversing_cnt = 3;
    }

    if (mc->reversing_cnt > 0) {
        mc->outputPWM = 0;
        mc->reversing_cnt--;
        return;
    }

    mc->prevSetRPM = mc->setRPM;

    // PID + FeedForward
    mc->offsetPWM  = PID_compute(&mc->pid, mc->setRPM, mc->currentRPM, dt);
    int ffPWM      = _getFeedForwardPWM(mc, mc->setRPM);
    mc->outputPWM  = ffPWM + (int)mc->offsetPWM;
    mc->outputPWM  = constrain(mc->outputPWM, -1023, 1023);

    // 정지 명령 시 완전 정지
    if (mc->setRPM == 0 && fabs(mc->currentRPM) < 2.0)
        mc->outputPWM = 0;
}

// ── 모터 출력 ─────────────────────────────────────────────────────────────────
void MotorControl_applyOutput(MotorControl *mc)
{
    if (mc->outputPWM != 0)
        DCMotorPWM_setSpeed(&mc->motor, mc->outputPWM);
    else
        DCMotorPWM_brake(&mc->motor);
}

void MotorControl_setTargetRPM(MotorControl *mc, double rpm) {
  mc->targetRPM = rpm;
}

long MotorControl_getEncoderPos(MotorControl *mc) {
  return Encoder_getPos(&mc->encoder);
}

double MotorControl_getCurrentRPM(MotorControl *mc) {
  return mc->currentRPM;
}