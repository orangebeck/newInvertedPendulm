#ifndef __PID_CORE_H__
#define __PID_CORE_H__

#include <stdio.h>
#include <stdlib.h>

typedef struct PIDController {
    double Kp; // 比例增益
    double Ki; // 积分增益
    double Kd; // 微分增益
    double prevError; // 上一次的误差
    double integral; // 误差积分

    double outMax; // Maximum output value
    double integralMax;
} PIDController;

void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double outMax, double integralMax);

double PID_Compute(PIDController* pid, double setpoint, double measuredValue, double dt);

#endif