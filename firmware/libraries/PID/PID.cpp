#include "PID.h"
#include <Arduino.h>

void PID_init(PID *pid,
              double kp, double ki, double kd,
              double minOutput, double maxOutput,
              double alpha)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->minOutput = minOutput;
    pid->maxOutput = maxOutput;
    pid->alpha     = alpha;
    PID_reset(pid);
}

void PID_reset(PID *pid)
{
    pid->prevInput        = 0;
    pid->cumError         = 0;
    pid->filteredRateError = 0;
}

double PID_compute(PID *pid, double setPoint, double input, double dt)
{
    if (setPoint == 0) {
        PID_reset(pid);
        return 0;
    }

    double error = setPoint - input;

    double fabsSet = fabs(setPoint);
    // 적분항
    double adaptiveKi = (fabsSet < 5) ? pid->ki * 0.3 : pid->ki;
    pid->cumError += error * dt;

    // Anti-windup
    double iLimit = pid->maxOutput / (2 * max(adaptiveKi, 0.0001));
    pid->cumError = constrain(pid->cumError, -iLimit, iLimit);

    // 미분항 노이즈 필터
    double rateErrorRaw    = -(input - pid->prevInput) / dt;
    pid->filteredRateError = pid->alpha * rateErrorRaw
                           + (1 - pid->alpha) * pid->filteredRateError;
    pid->prevInput = input;

    double output = pid->kp * error
                  + adaptiveKi * pid->cumError
                  + pid->kd * pid->filteredRateError;

    return constrain(output, pid->minOutput, pid->maxOutput);
}