#include "PSO.h"

double WIteration(int iteration)
{
    return WMAX - (WMAX - WMIN) * iteration / MAX_ITERATIONS;
}

double ITAE(double time, double target, double current, double samplingTime)
{
    return time * fabs(target - current) * samplingTime;
}

void backupPID(PSO *p, CtrlParams *pid)
{
    if (pid)
        p->cpid = pid;
    p->origin_cpid.Kp = p->cpid->Kp;
    p->origin_cpid.Ki = p->cpid->Ki;
    p->origin_cpid.Kd = p->cpid->Kd;
}

void restorePID(PSO *p)
{
    if (!p || !p->pid)
        return;
    p->cpid->Kp = p->origin_cpid.Kp;
    p->cpid->Ki = p->origin_cpid.Ki;
    p->cpid->Kd = p->origin_cpid.Kd;
}


double fitness(PSO *p, int i, double target, double ITAETime, double PIDSamplingTime)
{
    double distance = 0.0;
    double time_ = 0;
    backupPID(p, NULL);
    // p->pid->integral = 0;
    // p->pid->prevError = 0;
    p->cpid->Kp = p->particles[i].position[0];
    p->cpid->Ki = p->particles[i].position[1];
    p->cpid->Kd = p->particles[i].position[2];
    double ret = 0;
    double ITAERet = 0;
    pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
    control_xmt_module_infoSt->foundation_zero = control_xmt_module_infoSt->target + distance; // 暂定增加100微米
    pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
    for (time_ = 0; time_ < (ITAETime / PIDSamplingTime); time_++)
    {
        pthread_mutex_lock(&control_cl_module_infoSt->mutex);
        pthread_cond_wait(&control_cl_module_infoSt->cond, &control_cl_module_infoSt->mutex);
        ret = control_cl_module_infoSt->clData;
        pthread_mutex_unlock(&control_cl_module_infoSt->mutex);
        // if(fabs(0.0 - distance) >= 0.0000001)
        // {
        //     if (fabs(ret - control_xmt_module_infoSt->target + distance) > 3 * distance)
        //     {
        //         ITAERet = DBL_MAX;
        //         break;
        //     }
        // }
            ITAERet += ITAE(time_, control_xmt_module_infoSt->foundation_zero, ret, PIDSamplingTime);
    }
    pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
    control_xmt_module_infoSt->foundation_zero = control_xmt_module_infoSt->foundation_zero - distance; // 暂定增加100微米
    pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
    if (fabs(ret - (control_xmt_module_infoSt->foundation_zero + distance)) > 0.01)
    {
        ITAERet = DBL_MAX; // 如果没有稳定在目标值附近，则认为适应度为无穷大
        LOG(LOG_INFO, "[PSO] fitness: Iterations = %d, ITAE = INF,  Kp = %f, Ki = %f, Kd = %f\n",
               i,
               p->particles[i].position[0],
               p->particles[i].position[1],
               p->particles[i].position[2]);
    }
    else
    {
        LOG(LOG_INFO, "[PSO] fitness: Iterations = %d, ITAE = %f,  Kp = %f, Ki = %f, Kd = %f\n",
               i,
               ITAERet,
               p->particles[i].position[0],
               p->particles[i].position[1],
               p->particles[i].position[2]);
    }

    LOG(LOG_INFO, "[PSO] [check] target = %f, cur = %f, minus = %f\n", control_xmt_module_infoSt->foundation_zero + distance, ret, fabs(ret - (control_xmt_module_infoSt->foundation_zero + distance)));
    restorePID(p);

    // 等待恢复到最开始的基准
    double tmp;
    time_t start_time = 0;
    while (1)
    {
        double diff = fabs(control_cl_module_infoSt->clData - control_xmt_module_infoSt->foundation_zero);
        if (diff <= 0.005)
        {
            if (start_time == 0)
            {
                start_time = time(NULL);
            }
            else if (time(NULL) - start_time >= HOLD_SECONDS)
            {
                break; // 已经稳定超过 HOLD_SECONDS 秒
            }
        }
        else
        {
            start_time = 0; // 条件被打破，重新计时
        }
        sleep(1); // 每秒检查一次
    }

    LOG(LOG_INFO, "[PSO] fitness over!\n");

    return ITAERet;
}

double V_MAX[] = {0.1, 0.001, 5}; // 每个维度的最大速度
double Origin_PID[3] = {0};       // 原始PID参数
int updateParticle(PSO *p)
{
    Origin_PID[0] = p->origin_pid.Kp;
    Origin_PID[1] = p->origin_pid.Ki;
    Origin_PID[2] = p->origin_pid.Kd;
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        // LOG(LOG_INFO, "Particle %d: Before Position = (%f, %f, %f), Velocity = (%f, %f, %f)\n", i, p->particles[i].position[0], p->particles[i].position[1], p->particles[i].position[2], p->particles[i].velocity[0], p->particles[i].velocity[1], p->particles[i].velocity[2]);
        for (int j = 0; j < DIMENSIONS; j++)
        {
            int count = 0; // 用于计数，确保粒子位置与原始PID参数同号
            double tmp_v = 0, tmp_p = p->particles[i].position[j];
            do
            {
                tmp_v = 0, tmp_p = p->particles[i].position[j];
                tmp_v = WIteration(p->iteration) * p->particles[i].velocity[j] + C1 * ((double)rand() / RAND_MAX) * (p->particles[i].bestPosition[j] - p->particles[i].position[j]) + C2 * ((double)rand() / RAND_MAX) * (p->globalBestPosition[j] - p->particles[i].position[j]);

                // LOG(LOG_INFO, "Particle %d: p->particles[i].bestPosition[j] = %f, p->particles[i].position[j] = %f, p->globalBestPosition[i] = %f\n", i, p->particles[i].bestPosition[j], p->particles[i].position[j], p->globalBestPosition[j]);
                // LOG(LOG_INFO, "Particle %d: p->particles[i].bestPosition[j] - p->particles[i].position[j] = %f, p->globalBestPosition[i] - p->particles[i].position[j] = %f\n", i, p->particles[i].bestPosition[j] - p->particles[i].position[j], p->globalBestPosition[j] - p->particles[i].position[j]);

                if (tmp_v > V_MAX[j])
                {
                    tmp_v = V_MAX[j];
                }
                else if (tmp_v < -V_MAX[j])
                {
                    tmp_v = -V_MAX[j];
                }
                tmp_p += tmp_v;
                count++;
            } while (tmp_p * Origin_PID[j] < 0 && count < 100); // 确保粒子位置与原始PID参数同号

            p->particles[i].velocity[j] = tmp_v; // 更新速度
            p->particles[i].position[j] = tmp_p; // 更新位置
        }

        p->particles[i].fitness = fitness(p, i, p->target, p->ITAETime, p->PIDSamplingTime);
        if (p->particles[i].fitness < p->particles[i].bestFitness)
        {
            for (int j = 0; j < DIMENSIONS; j++)
            {
                p->particles[i].bestPosition[j] = p->particles[i].position[j];
            }
            p->particles[i].bestFitness = p->particles[i].fitness;
        }
        if (p->particles[i].bestFitness < p->globalBestFitness)
        {
            for (int j = 0; j < DIMENSIONS; j++)
            {
                p->globalBestPosition[j] = p->particles[i].position[j];
            }
            p->globalBestFitness = p->particles[i].bestFitness;
        }
        // LOG(LOG_INFO, "Particle %d: After Position = (%f, %f, %f), Velocity = (%f, %f, %f)\n", i, p->particles[i].position[0], p->particles[i].position[1], p->particles[i].position[2], p->particles[i].velocity[0], p->particles[i].velocity[1], p->particles[i].velocity[2]);
    }
    return 0;
}

int initPSO(PSO *pso)
{
    pso->globalBestFitness = DBL_MAX;
    // 初始化粒子群
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        if (i == 0)
        {
            pso->particles[i].position[0] = pso->origin_cpid.Kp; // 粒子在[-0.5,0.5]之间随机初始化
            pso->particles[i].velocity[0] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_P - PSO_INIT_INDEX_P / 2;

            pso->particles[i].position[1] = pso->origin_cpid.Ki; // 粒子在[-0.5,0.5]之间随机初始化
            pso->particles[i].velocity[1] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_I - PSO_INIT_INDEX_I / 2;

            pso->particles[i].position[2] = pso->origin_cpid.Kd; // 粒子在[-0.5,0.5]之间随机初始化
            pso->particles[i].velocity[2] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_D - PSO_INIT_INDEX_D / 2;
        }
        else
        {
            do
            {
                pso->particles[i].position[0] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_P - PSO_INIT_INDEX_P / 2 + pso->origin_cpid.Kp; // 粒子在[-0.5,0.5]之间随机初始化
                pso->particles[i].velocity[0] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_P - PSO_INIT_INDEX_P / 2;
            } while ((pso->particles[i].position[0] * pso->origin_pid.Kp) > 0);

            do
            {
                pso->particles[i].position[1] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_I - PSO_INIT_INDEX_I / 2 + pso->origin_cpid.Ki; // 粒子在[-0.5,0.5]之间随机初始化
                pso->particles[i].velocity[1] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_I - PSO_INIT_INDEX_I / 2;
            } while ((pso->particles[i].position[1] * pso->origin_pid.Ki) >0);

            do
            {
                pso->particles[i].position[2] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_D - PSO_INIT_INDEX_D / 2 + pso->origin_cpid.Kd; // 粒子在[-0.5,0.5]之间随机初始化
                pso->particles[i].velocity[2] = (double)rand() / RAND_MAX * PSO_INIT_INDEX_D - PSO_INIT_INDEX_D / 2;
            } while ((pso->particles[i].position[2] * pso->origin_pid.Kd) > 0);
        }

        for (int j = 0; j < DIMENSIONS; j++)
        {
            pso->particles[i].bestPosition[j] = pso->particles[i].position[j];
        }
        pso->particles[i].fitness = fitness(pso, i, pso->target, pso->ITAETime, pso->PIDSamplingTime);
        pso->particles[i].bestFitness = pso->particles[i].fitness;
        if (pso->particles[i].bestFitness < pso->globalBestFitness)
        {
            for (int j = 0; j < DIMENSIONS; j++)
            {
                pso->globalBestPosition[j] = pso->particles[i].position[j];
            }
            pso->globalBestFitness = pso->particles[i].bestFitness;
        }
    }
    return 0;
}