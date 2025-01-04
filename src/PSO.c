#include "PSO.h"

double WIteration(int iteration)
{
    return WMAX - (WMAX - WMIN) * iteration / MAX_ITERATIONS;
}

double ITAE(double time, double target, double current, double samplingTime)
{
    return time * fabs(target - current) * samplingTime;
}

double fitness(PSO *p, int i, double target, double ITAETime, double PIDSamplingTime)
{
    double time = 0;
    p->pid.integral = 0;
    p->pid.prevError = 0;
    p->pid.Kp = p->particles[i].position[0];
    p->pid.Ki = p->particles[i].position[1];
    p->pid.Kd = p->particles[i].position[2];
    double ret = 0;
    double ITAERet = 0;
    pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
    control_xmt_module_infoSt->foundation_zero = control_xmt_module_infoSt->foundation_zero + 0.1;  //暂定增加100微米
    pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
    for (time = 0; time < ITAETime; time++)
    {
        pthread_mutex_lock(&control_cl_module_infoSt->mutex);
        pthread_cond_wait(&control_cl_module_infoSt->cond, &control_cl_module_infoSt->mutex); 
        ret = control_cl_module_infoSt->clData;
        pthread_mutex_unlock(&control_cl_module_infoSt->mutex);
        ITAERet += ITAE(time, target, ret, PIDSamplingTime);
    }
    pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
    control_xmt_module_infoSt->foundation_zero = control_xmt_module_infoSt->foundation_zero - 0.1;  //暂定增加100微米
    pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
    if( fabs(ret - target) > 0.01 )
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
        pso->particles[i].fitness = fitness(pso, i, pso->target, pso->ITAETime, pso->PIDSamplingTime);
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