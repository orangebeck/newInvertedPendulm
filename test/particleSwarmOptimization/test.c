#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define MAX_PARTICLES 30
#define MAX_ITERATIONS 100
#define W 0.5    // 惯性权重
#define C1 1.5   // 认知学习因子
#define C2 1.5   // 社会学习因子

// PID控制器结构体
typedef struct {
    double Kp;  // 比例增益
    double Ki;  // 积分增益
    double Kd;  // 微分增益
    double best_Kp;  // 最佳比例增益
    double best_Ki;  // 最佳积分增益
    double best_Kd;  // 最佳微分增益
    double best_error;  // 最佳误差
    double velocity_Kp; // 速度（调整速度）
    double velocity_Ki; 
    double velocity_Kd;
} Particle;

// 全局最优粒子
Particle global_best;

double target_output = 100.0;  // 目标输出（假设我们想控制一个过程到这个值）

// 目标函数（此处为PID控制误差）
double target_function(double Kp, double Ki, double Kd) {
    double error = 0.0;
    double set_point = 100.0; // 目标设定点
    double process_variable = 0.0; // 实际过程变量（初始为0）

    // 模拟一个简单的被控系统过程，这里只是简单的示例，你可以根据实际情况修改
    double sum_error = 0.0;
    for (int i = 0; i < 1000; i++) {
        double error_signal = set_point - process_variable;
        sum_error += error_signal;
        double d_error = error_signal - sum_error;

        // PID控制公式
        process_variable += Kp * error_signal + Ki * sum_error + Kd * d_error;

        // 计算误差
        error += fabs(error_signal); // 误差为目标与实际值的绝对差
    }

    return error;  // 返回误差
}

// 粒子群优化算法
void pso() {
    Particle particles[MAX_PARTICLES];
    
    // 初始化粒子
    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].Kp = (rand() % 100) / 100.0 * 10.0;  // 初始化Kp
        particles[i].Ki = (rand() % 100) / 100.0 * 10.0;  // 初始化Ki
        particles[i].Kd = (rand() % 100) / 100.0 * 10.0;  // 初始化Kd
        particles[i].velocity_Kp = 0.0;  // 初始速度为0
        particles[i].velocity_Ki = 0.0;
        particles[i].velocity_Kd = 0.0;
        particles[i].best_Kp = particles[i].Kp;
        particles[i].best_Ki = particles[i].Ki;
        particles[i].best_Kd = particles[i].Kd;
        particles[i].best_error = target_function(particles[i].Kp, particles[i].Ki, particles[i].Kd);
        
        // 初始化全局最佳粒子
        if (i == 0 || particles[i].best_error < global_best.best_error) {
            global_best = particles[i];
        }
    }

    // 迭代更新粒子
    for (int t = 0; t < MAX_ITERATIONS; t++) {
        for (int i = 0; i < MAX_PARTICLES; i++) {
            // 更新速度
            particles[i].velocity_Kp = W * particles[i].velocity_Kp
                + C1 * (rand() / (double) RAND_MAX) * (particles[i].best_Kp - particles[i].Kp)
                + C2 * (rand() / (double) RAND_MAX) * (global_best.best_Kp - particles[i].Kp);
            particles[i].velocity_Ki = W * particles[i].velocity_Ki
                + C1 * (rand() / (double) RAND_MAX) * (particles[i].best_Ki - particles[i].Ki)
                + C2 * (rand() / (double) RAND_MAX) * (global_best.best_Ki - particles[i].Ki);
            particles[i].velocity_Kd = W * particles[i].velocity_Kd
                + C1 * (rand() / (double) RAND_MAX) * (particles[i].best_Kd - particles[i].Kd)
                + C2 * (rand() / (double) RAND_MAX) * (global_best.best_Kd - particles[i].Kd);
            
            // 更新粒子位置
            particles[i].Kp += particles[i].velocity_Kp;
            particles[i].Ki += particles[i].velocity_Ki;
            particles[i].Kd += particles[i].velocity_Kd;

            // 目标函数评估
            double error = target_function(particles[i].Kp, particles[i].Ki, particles[i].Kd);
            
            // 如果当前粒子的误差更小，更新其最佳解
            if (error < particles[i].best_error) {
                particles[i].best_error = error;
                particles[i].best_Kp = particles[i].Kp;
                particles[i].best_Ki = particles[i].Ki;
                particles[i].best_Kd = particles[i].Kd;
            }

            // 更新全局最佳粒子
            if (error < global_best.best_error) {
                global_best = particles[i];
            }
        }

        printf("Iteration %d: Best Kp: %.4f, Ki: %.4f, Kd: %.4f, Error: %.4f\n", t, global_best.best_Kp, global_best.best_Ki, global_best.best_Kd, global_best.best_error);
    }
}

int main() {
    srand(time(NULL));
    
    // 运行粒子群优化
    pso();
    
    // 输出最优PID参数
    printf("\nOptimized PID parameters:\n");
    printf("Kp: %.4f\n", global_best.best_Kp);
    printf("Ki: %.4f\n", global_best.best_Ki);
    printf("Kd: %.4f\n", global_best.best_Kd);
    
    return 0;
}
