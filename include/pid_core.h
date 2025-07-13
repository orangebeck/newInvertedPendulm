#ifndef __PID_CORE_H__
#define __PID_CORE_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct PIDController {
    double Kp; // 比例增益
    double Ki; // 积分增益
    double Kd; // 微分增益
    double prevError;      // e(t-1)
    double prevPrevError;  // e(t-2)
    double lastOutput;     // u(t-1)
    double integral; // 误差积分

    double outMax; // Maximum output value
    double integralMax;

    double ratio;

    double deadzone;
} PIDController;

void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double outMax, double integralMax);

double PID_Compute(PIDController* pid, double setpoint, double measuredValue, double dt);

#endif