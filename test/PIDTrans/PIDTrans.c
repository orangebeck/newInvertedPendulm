#include <stdio.h>
#include <math.h>

// 精确PID控制器结构体
typedef struct {
    float kp;             // 比例增益
    float ki;             // 积分增益
    float kd;             // 微分增益
    float setpoint;       // 设定值
    float prev_error;     // 上一个误差
    float integral;       // 积分项
    float prev_measurement; // 上一个测量值（用于DoM）
    float output;         // 输出值
    float max_output;     // 最大输出
    float min_output;     // 最小输出
    float max_integral;   // 积分项最大值，防止积分风up
} PIDController;

// 初始化PID控制器
void PID_Init(PIDController* pid, float kp, float ki, float kd, float max_output, float min_output, float max_integral, double ratio) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->max_integral = max_integral;
}

// PID计算函数
float PID_Compute(PIDController* pid, float actual_value) {
    // 计算误差
    float error = pid->setpoint - actual_value;

    // 积分项计算
    pid->integral += error;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;  // 防止积分过大
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral; // 防止积分过小
    }

    // 微分项计算，采用测量值的微分（Derivative on Measurement）
    float derivative = actual_value - pid->prev_measurement;

    // PID输出计算
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 防止输出超出限制
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }

    // 更新历史测量值和误差
    pid->prev_measurement = actual_value;

    return pid->output;
}

// 模拟系统状态的更新（例如电机控制中的速度更新）
void system_update(float* current_value, float control_signal) {
    // 假设控制信号直接影响系统的状态，简单模拟
    *current_value += control_signal * 0.1f;  // 控制信号影响系统（例如速度）
}

int main() {
    // 初始化PID控制器，假设最大输出为100，最小输出为-100，最大积分为50
    PIDController pid;
    PID_Init(&pid, 2.0f, 0.1f, 0.05f, 100.0f, -100.0f, 50.0f);

    // 假设目标位置（设定值）是10，当前值是0
    float current_value = 0.0f;
    float target_value = 10.0f;

    pid.setpoint = target_value;  // 设置目标位置

    // 运行PID控制过程，模拟控制器调整系统状态
    for (int i = 0; i < 50; i++) {
        printf("Iteration: %d\n", i);
        float control_signal = PID_Compute(&pid, current_value);  // 计算控制信号
        system_update(&current_value, control_signal);  // 更新系统状态
        printf("Current Value: %.2f, Control Signal: %.2f, Output: %.2f\n", current_value, control_signal, pid.output);
    }

    return 0;
}
