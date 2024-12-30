#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include "pid_core.h"

#define NUM_PARTICLES 30   // 粒子数量
#define DIMENSIONS 3      // 搜索空间的维度
#define MAX_ITERATIONS 30 // 最大迭代次数
#define WMAX 1.0              // 惯性权重
#define WMIN 0.1              // 局部权重
#define C1 1.5             // 自我认知权重
#define C2 1.5             // 社会认知权重

#define TARGET 10.0        // PID控制目标值
#define ITAETIME 10.0        // ITAE积分时间
#define PIDSAMPLINGTIME 1  // PID采样时间

typedef struct {
    double position[DIMENSIONS];  // 粒子的位置
    double velocity[DIMENSIONS];  // 粒子的速度
    double fitness;  // 粒子的适应度
    double bestPosition[DIMENSIONS];  // 粒子的历史最优位置
    double bestFitness;  // 粒子的历史最优值
} Particle;

typedef struct {
    PIDController pid;
    Particle particles[NUM_PARTICLES];
    double globalBestPosition[DIMENSIONS];
    double globalBestFitness;
    int iteration;
} PSO;

double WIteration(int iteration)
{
    return WMAX - (WMAX - WMIN) * iteration / MAX_ITERATIONS;
}

double ITAE(double time, double target, double current, double samplingTime)
{
    return time * fabs(target - current) * samplingTime;
}

double fitness(PSO *p, int i)
{
    double time = 0;
    p->pid.integral = 0;
    p->pid.prevError = 0;
    p->pid.Kp = p->particles[i].position[0];
    p->pid.Ki = p->particles[i].position[1];
    p->pid.Kd = p->particles[i].position[2];
    double ret = 0;
    double ITAERet = 0;
    for (time = 0; time < ITAETIME; time++)
    {
        ITAERet += ITAE(time, TARGET, ret, PIDSAMPLINGTIME);
        ret += PID_Compute(&(p->pid), TARGET, ret, PIDSAMPLINGTIME);
    }
    if( fabs(ret - TARGET) > 0.1 )
    {
        ITAERet = DBL_MAX;
    }
    return ITAERet;
}

int updateParticle(PSO *p)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        for(int j = 0; j < DIMENSIONS; j++)
        {
            p->particles[i].velocity[j] = WIteration(p->iteration) * p->particles[i].velocity[j] + C1 * rand() / RAND_MAX * (p->particles[i].bestPosition[j] - p->particles[i].position[j]) + C2 * rand() / RAND_MAX * (p->globalBestPosition[i] - p->particles[i].position[j]);
            p->particles[i].position[j] += p->particles[i].velocity[j];
        }
        p->particles[i].fitness = fitness(p, i);
        if (p->particles[i].fitness < p->particles[i].bestFitness) {
            for (int j = 0; j < DIMENSIONS; j++) {
                p->particles[i].bestPosition[j] = p->particles[i].position[j];
            }
            p->particles[i].bestFitness = p->particles[i].fitness;
        }
        if (p->particles[i].bestFitness < p->globalBestFitness) {
            for (int j = 0; j < DIMENSIONS; j++) {
                p->globalBestPosition[j] = p->particles[i].position[j];
            }
            p->globalBestFitness = p->particles[i].bestFitness;
        }
    }
    return 0;
    
}

int initPSO(PSO *pso) {
    pso->globalBestFitness = DBL_MAX;
    // 初始化粒子群
    for (int i = 0; i < NUM_PARTICLES; i++) {
        for (int j = 0; j < DIMENSIONS; j++) {
            pso->particles[i].position[j] = (double)rand() / RAND_MAX * 1 - 0.5; //粒子在[-0.5,0.5]之间随机初始化
            pso->particles[i].velocity[j] = (double)rand() / RAND_MAX * 1 - 0.5;
        }
        for (int j = 0; j < DIMENSIONS; j++) {
            pso->particles[i].bestPosition[j] = pso->particles[i].position[j];
        }
        pso->particles[i].fitness = fitness(pso, i);
        pso->particles[i].bestFitness = pso->particles[i].fitness;
        if (pso->particles[i].bestFitness < pso->globalBestFitness) {
            for (int j = 0; j < DIMENSIONS; j++) {
                pso->globalBestPosition[j] = pso->particles[i].position[j];
            }
            pso->globalBestFitness = pso->particles[i].bestFitness;
        }
    }
    return 0;
}

int main() {
    PSO pso;
    srand(time(NULL));

    PID_Init(&(pso.pid), 1.0, 0.0, 0.0, 10, 10);
    initPSO(&pso);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        pso.iteration = i;
        updateParticle(&pso);
        printf("Iteration %d: Best fitness = %f, Kp = %f, Ki = %f, Kd = %f\n", i, pso.globalBestFitness, pso.globalBestPosition[0], pso.globalBestPosition[1], pso.globalBestPosition[2]);
    }

    //模拟pid进行计算
    double i = 0;
    double ret = 0;
    pso.pid.integral = 0;
    pso.pid.prevError = 0;
    pso.pid.Kp = pso.globalBestPosition[0];
    pso.pid.Ki = pso.globalBestPosition[1];
    pso.pid.Kd = pso.globalBestPosition[2];
    for (i = 0; i < MAX_ITERATIONS; i++)
    {
        ret += PID_Compute(&(pso.pid), TARGET, ret, PIDSAMPLINGTIME);
        printf("Iteration %lf: PID output = %lf\n", i, ret);
    }
    
    return 0;
}