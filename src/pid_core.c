#include "pid_core.h"

void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double outMax, double integralMax)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prevError = 0.0;
    pid->integral = 0.0;
    pid->outMax = outMax; 
    pid->integralMax = integralMax; 
    pid->ratio = 1;
    pid->deadzone = 0.002;
}

double PID_Compute(PIDController *pid, double setpoint, double measuredValue, double dt)
{
    // printf("[PID_Compute] setpoint = %f, measuredValue = %f\n", setpoint, measuredValue);
    if(pid->Kp == 0 && pid->Ki == 0 && pid->Kp == 0) return 0.0;
    
    double error = setpoint - measuredValue;
    if(error > 20 || error <-20) return 0;
    double integral = pid->integral + error * dt;
    if (integral > pid->integralMax) {
        pid->integral = pid->integralMax;
    } else if (integral < -pid->integralMax) {
        pid->integral = -pid->integralMax;
    }else
    {
        pid->integral = integral;
    }
    double derivative = (error - pid->prevError) / dt;
    double output = 0.0;

        output = measuredValue + pid->Kp *  error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->outMax) {
        output = pid->outMax;
    } else if (output < -pid->outMax) {
        output = -pid->outMax;
    }

    // Update state
    pid->prevError = error;

    return output;
}


// double PID_Compute(PIDController *pid, double setpoint, double measuredValue, double dt)
// {
//     if(pid->Kp == 0 && pid->Ki == 0 && pid->Kd == 0) return 0.0;

//     double error = setpoint - measuredValue;

//     // 限制误差范围（可以根据需要保留或删掉）
//     if(error > 20 || error < -20) return 0;

//     // 计算增量式 PID 输出增量 Δu
//     double deltaOutput = 0.0;

//     double diffError = error - pid->prevError; // e(t) - e(t-1)
//     double secondDiff = error - 2 * pid->prevError + pid->prevPrevError; // e(t) - 2e(t-1) + e(t-2)

//     deltaOutput = pid->Kp * diffError
//                 + pid->Ki * error * dt
//                 + pid->Kd * secondDiff / dt;

//     // 加到上一次的输出
//     double output = pid->lastOutput + deltaOutput;

//     // 限幅
//     if (output > pid->outMax) {
//         output = pid->outMax;
//     } else if (output < -pid->outMax) {
//         output = -pid->outMax;
//     }

//     // 更新状态
//     pid->prevPrevError = pid->prevError;
//     pid->prevError = error;
//     pid->lastOutput = output;

//     return output;
// }