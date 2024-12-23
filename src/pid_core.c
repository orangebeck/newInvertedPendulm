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
}

double PID_Compute(PIDController *pid, double setpoint, double measuredValue, double dt)
{
    if(pid->Kp == 0 && pid->Ki == 0 && pid->Kp == 0) return 0.0;
    
    double error = setpoint - measuredValue;
    if(error > 2 || error <-2) return 0;
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

    double output = pid->Kp * pid->ratio * error + pid->Ki * pid->ratio * integral + pid->Kd * pid->ratio * derivative;

    if (output > pid->outMax) {
        output = pid->outMax;
    } else if (output < -pid->outMax) {
        output = -pid->outMax;
    }

    // Update state
    pid->prevError = error;

    printf("\nerror = %f\t", error);
    printf("pid->integral = %f\t", pid->integral);
    printf("derivative = %f\n", derivative);

    return output;
}