#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

#define NUM_PARTICLES 30   // 粒子数量
#define DIMENSIONS 1      // 搜索空间的维度
#define MAX_ITERATIONS 1000 // 最大迭代次数
#define W 0.5              // 惯性权重
#define C1 1.5             // 自我认知权重
#define C2 1.5             // 社会认知权重

// 粒子结构体
typedef struct {
    double position[DIMENSIONS];  // 粒子的位置
    double velocity[DIMENSIONS];  // 粒子的速度
    double best_position[DIMENSIONS];  // 粒子的历史最优位置
    double best_value;  // 粒子的历史最优值
} Particle;


typedef struct {
    struct Particle{
        double position[DIMENSIONS];  // 粒子的位置
        double velocity[DIMENSIONS];  // 粒子的速度
        double best_position[DIMENSIONS];  // 粒子的历史最优位置
        double best_value;  // 粒子的历史最优值
    }particle; 
    double global_best_position[DIMENSIONS];    //全局最优解
    double global_best_value;                   //全局最优值
} PSO;



//ITAE = ∑(0-∞) t|e(t)|dt
double ITAE(double (*system_output)(double, double, double), double *input, int num_samples, double Tau) {
    double ITAE_value = 0.0;
    for (int i = 1; i < num_samples; i++) {
        // 计算系统的输出
        double t = (double)i;
        double output = system_output(input[i], t, Tau);
        
        // 假设期望输出为 1
        double error = fabs(1 - output); // 期望输出是 1，误差为绝对值
        ITAE_value += t * error; // 时间加权绝对误差
    }
    return ITAE_value;
}

// 目标函数：求解 f(x) = x^2 的最小值
double objective_function(double position[DIMENSIONS]) {
    return position[0] * position[0];  // 目标函数 f(x) = x^2
}

// 更新粒子的位置和速度
void update_particle(Particle *particle, double global_best_position[DIMENSIONS]) {
    // 更新速度
    for (int i = 0; i < DIMENSIONS; i++) {
        double r1 = (double)rand() / RAND_MAX;
        double r2 = (double)rand() / RAND_MAX;
        particle->velocity[i] = W * particle->velocity[i] +
                               C1 * r1 * (particle->best_position[i] - particle->position[i]) +
                               C2 * r2 * (global_best_position[i] - particle->position[i]);
    }

    // 更新位置
    for (int i = 0; i < DIMENSIONS; i++) {
        particle->position[i] += particle->velocity[i];
    }

    // 计算当前值
    double current_value = objective_function(particle->position);
    if (current_value < particle->best_value) {
        // 如果当前解更优，则更新粒子的历史最优解
        for (int i = 0; i < DIMENSIONS; i++) {
            particle->best_position[i] = particle->position[i];
        }
        particle->best_value = current_value;
    }
}

// 初始化粒子群
void initialize_particles(Particle particles[NUM_PARTICLES], double global_best_position[DIMENSIONS], double *global_best_value) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        // 初始化位置和速度
        for (int j = 0; j < DIMENSIONS; j++) {
            particles[i].position[j] = ((double)rand() / RAND_MAX) * 10 - 5;  // 随机位置在 [-5, 5] 范围内
            particles[i].velocity[j] = ((double)rand() / RAND_MAX) * 2 - 1;  // 随机速度在 [-1, 1] 范围内
        }

        // 计算目标函数值
        particles[i].best_value = objective_function(particles[i].position);
        for (int j = 0; j < DIMENSIONS; j++) {
            particles[i].best_position[j] = particles[i].position[j];
        }

        // 更新全局最优解
        if (particles[i].best_value < *global_best_value) {
            *global_best_value = particles[i].best_value;
            for (int j = 0; j < DIMENSIONS; j++) {
                global_best_position[j] = particles[i].best_position[j];
            }
        }
    }
}

int main() {
    srand(time(NULL)); // 设置随机数种子

    // 初始化粒子群
    Particle particles[NUM_PARTICLES];
    double global_best_position[DIMENSIONS];
    double global_best_value = DBL_MAX; // 初始化为最大浮点数

    // 初始化粒子群
    initialize_particles(particles, global_best_position, &global_best_value);

    // 迭代更新粒子位置
    for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
        for (int i = 0; i < NUM_PARTICLES; i++) {
            update_particle(&particles[i], global_best_position);
        }

        // 每次迭代后，更新全局最优解
        for (int i = 0; i < NUM_PARTICLES; i++) {
            if (particles[i].best_value < global_best_value) {
                global_best_value = particles[i].best_value;
                for (int j = 0; j < DIMENSIONS; j++) {
                    global_best_position[j] = particles[i].best_position[j];
                }
            }
        }

        // 输出每隔100次迭代的结果
        if (iteration % 100 == 0) {
            printf("Iteration %d: Best Value = %f\n", iteration, global_best_value);
        }
    }

    // 输出最优解
    printf("Global Best Value = %f\n", global_best_value);
    printf("Global Best Position = ");
    for (int i = 0; i < DIMENSIONS; i++) {
        printf("%f ", global_best_position[i]);
    }
    printf("\n");

    return 0;
}
