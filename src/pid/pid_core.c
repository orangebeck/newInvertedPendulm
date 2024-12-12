#include "pid_core.h"

void PID_Init(PIDController *pid, double Kp, double Ki, double Kd, double outMin, double outMax)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prevError = 0.0;
    pid->integral = 0.0;
    pid->outMin = outMin; // Minimum output value
    pid->outMax = outMax; // Maximum output value
}

double PID_Compute(PIDController *pid, double setpoint, double measuredValue, double dt)
{
    if(pid->Kp == 0 && pid->Ki == 0 && pid->Kp == 0) return 0.0;
    
    double error = setpoint - measuredValue;
    double integral = pid->integral + error * dt;
    double derivative = (error - pid->prevError) / dt;

    // Preliminary output without constraints
    double output = pid->Kp * error + pid->Ki * integral + pid->Kd * derivative;

    // Check for output saturation and adjust integral term if necessary
    if (output > pid->outMax)
    {
        output = pid->outMax;
        // Prevent integral term from increasing if output is saturated at the upper limit
        if (error > 0)
            integral = pid->integral;
    }
    else if (output < pid->outMin)
    {
        output = pid->outMin;
        // Prevent integral term from decreasing further if output is saturated at the lower limit
        if (error < 0)
            integral = pid->integral;
    }

    // Update state
    pid->prevError = error;
    pid->integral = integral;

    printf("\nerror = %f\t", error);
    printf("pid->integral = %f\t", pid->integral);
    printf("derivative = %f\n", derivative);

    return output;
}
