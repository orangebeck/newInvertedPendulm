#include "PSO.h"

double WIteration(int iteration)
{
    return WMAX - (WMAX - WMIN) * iteration / MAX_ITERATIONS;
}

double ITAE(double time, double target, double current, double samplingTime)
{
    return time * fabs(target - current) * samplingTime;
}

void backupPID(PSO *p, PIDController *pid)
{
    if(pid) p->pid = pid;
    p->origin_pid.Kp = p->pid->Kp;
    p->origin_pid.Ki = p->pid->Ki;
    p->origin_pid.Kd = p->pid->Kd;
}

void restorePID(PSO *p)
{
    if(!p || !p->pid) return;
    p->pid->Kp = p->origin_pid.Kp;
    p->pid->Ki = p->origin_pid.Ki;
    p->pid->Kd = p->origin_pid.Kd;
}


double fitness(PSO *p, int i, double target, double ITAETime, double PIDSamplingTime)
{
    double time_ = 0;
    backupPID(p, NULL);
    p->pid->integral = 0;
    p->pid->prevError = 0;
    p->pid->Kp = p->particles[i].position[0];
    p->pid->Ki = p->particles[i].position[1];
    p->pid->Kd = p->particles[i].position[2];
    double ret = 0;
    double ITAERet = 0;
    pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
    control_xmt_module_infoSt->foundation_zero = control_xmt_module_infoSt->foundation_zero + 0.1;  //暂定增加100微米
    pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
    for (time_ = 0; time_ < ITAETime; time_++)
    {
        pthread_mutex_lock(&control_cl_module_infoSt->mutex);
        pthread_cond_wait(&control_cl_module_infoSt->cond, &control_cl_module_infoSt->mutex); 
        ret = control_cl_module_infoSt->clData;
        pthread_mutex_unlock(&control_cl_module_infoSt->mutex);
        ITAERet += ITAE(time_, target, ret, PIDSamplingTime);
    }
    pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
    control_xmt_module_infoSt->foundation_zero = control_xmt_module_infoSt->foundation_zero - 0.1;  //暂定增加100微米
    pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
    if( fabs(ret - target) > 0.01 )
    {
        ITAERet = DBL_MAX;
    }
    restorePID(p);
    printf("PSO fitness: Iterations = %d, ITAE = %f,  Kp = %f, Ki = %f, Kd = %f\n", i, ITAERet,p->particles[i].position[0], p->particles[i].position[1], p->particles[i].position[2] );
    //等待恢复到最开始的基准
    double tmp;
    time_t start_time = 0;
    while (1) {
        double diff = fabs(control_cl_module_infoSt->clData - target);
        if (diff <= 0.01) {
            if (start_time == 0) {
                start_time = time(NULL);
            } else if (time(NULL) - start_time >= HOLD_SECONDS) {
                return 1; // 已经稳定超过 HOLD_SECONDS 秒
            }
        } else {
            start_time = 0; // 条件被打破，重新计时
        }
        sleep(1); // 每秒检查一次
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
        p->particles[i].fitness = fitness(p, i, p->target, p->ITAETime, p->PIDSamplingTime);
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

        pso->particles[i].position[0] = (double)rand() / RAND_MAX *PSO_INIT_INDEX - PSO_INIT_INDEX/2 + pso->origin_pid.Kp; //粒子在[-0.5,0.5]之间随机初始化
        pso->particles[i].velocity[0] = (double)rand() / RAND_MAX * PSO_INIT_INDEX - PSO_INIT_INDEX/2;

        pso->particles[i].position[1] = (double)rand() / RAND_MAX *PSO_INIT_INDEX - PSO_INIT_INDEX/2 + pso->origin_pid.Ki; //粒子在[-0.5,0.5]之间随机初始化
        pso->particles[i].velocity[1] = (double)rand() / RAND_MAX * PSO_INIT_INDEX - PSO_INIT_INDEX/2;

        pso->particles[i].position[2] = (double)rand() / RAND_MAX *PSO_INIT_INDEX - PSO_INIT_INDEX/2 + pso->origin_pid.Kd; //粒子在[-0.5,0.5]之间随机初始化
        pso->particles[i].velocity[2] = (double)rand() / RAND_MAX * PSO_INIT_INDEX - PSO_INIT_INDEX/2;

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