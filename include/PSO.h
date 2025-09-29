#ifndef PSO_H
#define PSO_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include "pid_core.h"
#include "control_cl_module.h"
#include "control_xmt_module.h"
#include "pend_ctrl.h"

#define NUM_PARTICLES 5  // 粒子数量
#define DIMENSIONS 3      // 搜索空间的维度
#define MAX_ITERATIONS 40 // 最大迭代次数
#define WMAX 0.5              // 惯性权重
#define WMIN 0.5              // 局部权重
#define C1 1.5             // 自我认知权重
#define C2 1.5             // 社会认知权重
#define PSO_INIT_INDEX_P 0.08
#define PSO_INIT_INDEX_I 0.01
#define PSO_INIT_INDEX_D 0.02
#define PSO_INIT_INDEX 0.0001
#define HOLD_SECONDS 5

typedef struct {
    double position[DIMENSIONS];  // 粒子的位置
    double velocity[DIMENSIONS];  // 粒子的速度
    double fitness;  // 粒子的适应度
    double bestPosition[DIMENSIONS];  // 粒子的历史最优位置
    double bestFitness;  // 粒子的历史最优值
} Particle;

typedef struct {
    PIDController *pid;
    PIDController origin_pid;
    CtrlParams *cpid;
    CtrlParams origin_cpid;
    Particle particles[NUM_PARTICLES];
    double globalBestPosition[DIMENSIONS];
    double globalBestFitness;
    int iteration;

    double target;
    double ITAETime;
    double PIDSamplingTime;

} PSO;

extern control_cl_module_info *control_cl_module_infoSt;
extern control_xmt_module_info *control_xmt_module_infoSt;

double WIteration(int iteration);

double ITAE(double time, double target, double current, double samplingTime);

double fitness(PSO *p, int i, double target, double ITAETime, double PIDSamplingTime);

int updateParticle(PSO *p);

int initPSO(PSO *pso);

void backupPID(PSO *p, CtrlParams *pid);

void restorePID(PSO *p);

#endif