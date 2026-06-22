#pragma once
 
typedef struct {
    double kp, ki, kd;
    double minOutput, maxOutput;
    double prevInput;
    double cumError;
    double filteredRateError;
    double alpha;
} PID;
 
void   PID_init(PID *pid,
                double kp, double ki, double kd,
                double minOutput, double maxOutput,
                double alpha);
void   PID_reset(PID *pid);
double PID_compute(PID *pid, double setPoint, double input, double dt);