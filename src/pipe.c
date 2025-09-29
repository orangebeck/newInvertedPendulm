#include "pipe.h"
#include "motor.h"

int  stopThreadFlag = 0;

char *send_path = "/home/zhouweijie/pipeToQT";
char *receive_path = "/home/zhouweijie/pipeToApp";
char *debug_path = "/home/zhouweijie/Debug";

motor motorList[] = {
    {
        .led = 4.0,
        .motor_ctrl = {
            .enx = 0,
            .micro = 400,
        },
    },
    {
        .led = 4.0,
        .motor_ctrl = {
            .enx = 1,
            .micro = 1600,
        },
    },
    {}
};

int motor_fd;

pipeShareData *initPipeShareDataSt(){
    pipeShareData *data = (pipeShareData *)malloc(sizeof(pipeShareData));

    if (data == NULL) {
        perror("Failed to allocate memory for pipeShareData");
        return NULL;
    }
    memset(data,0,sizeof(pipeShareData));

    PID_Init(&data->pid, 0, 0, 0, 0.15, 0.1);
    Ctrl_GetDefaultParams(0.05, &data->cpid);

    data->pid_status = 0;
    data->amplify_set = 0.15;

    if (pthread_mutex_init(&data->stop_mutex, NULL) != 0) {
        perror("Mutex initialization failed");
        free(data);
        return NULL;
    }

    if (pthread_mutex_init(&data->pso_mutex, NULL) != 0) {
        perror("Mutex initialization failed");
        free(data);
        return NULL;
    }

    if (pthread_mutex_init(&data->start_mutex, NULL) != 0) {
        perror("Mutex initialization failed");
        free(data);
        return NULL;
    }

    if (pthread_mutex_init(&data->write_mutex, NULL) != 0) {
        perror("Mutex initialization failed");
        free(data);
        return NULL;
    }

    if (pthread_cond_init(&data->stop_cond, NULL) != 0) {
        perror("Condition variable initialization failed");
        pthread_mutex_destroy(&data->stop_mutex);
        free(data);
        return NULL;
    }

    if (pthread_cond_init(&data->pso_cond, NULL) != 0) {
        perror("Condition variable initialization failed");
        pthread_mutex_destroy(&data->pso_mutex);
        free(data);
        return NULL;
    }

    if (pthread_cond_init(&data->start_cond, NULL) != 0) {
        perror("Condition variable initialization failed");
        pthread_mutex_destroy(&data->start_mutex);
        free(data);
        return NULL;
    }

    data->send_xmt_value = send_xmt_value_impl;
    data->send_xmt_command = send_xmt_command_impl;
    data->send_cl_value = send_cl_value_impl;
    data->send_cl_command = send_cl_command_impl;

    data->send_pid_error = send_pid_error_impl;
    data->send_pid_intergrate = send_pid_intergrate_impl;

    return data;
}

void destroyPipeShareDataSt(pipeShareData *data) {
    if (data != NULL) {
        pthread_mutex_destroy(&data->stop_mutex);
        pthread_cond_destroy(&data->stop_cond);
        free(data);
    }
}

int createPipe(char *path)
{
    if (access(path, F_OK) == -1)
    {
        if (mkfifo(path, 0666) != 0)
        {
            perror("mkfifo");
            return ERROR_PIPE_CREATE;
        }
        LOG(LOG_INFO, "Named pipe created at %s\n", path);
        return SUCCESS;
    }
    else
    {
        LOG(LOG_INFO, "Named pipe already exists at %s\n", path);
        return SUCCESS;
    }
}

int openPipe(char *path)
{
    char buffer[BUFFER_SIZE];
    int fd = open(path, O_RDWR);
    if (fd == -1)
    {
        perror("open");
        LOG(LOG_ERROR, "fail to open\n");
        return ERROR_PIPE_OPEN;
    }
    // while (read(fd, buffer, BUFFER_SIZE) > 0); // 清空缓冲区 这里面存在问题
    
    LOG(LOG_INFO, "Named pipe opened at %s, fd = %d\n", path, fd);
    return fd;
}

int parseString(char *string, char *infoList[], int *num)
{
    // 定义分割符为空格或tab
    const char *delimiters = " \t";
    char *token;
    int count = 0;

    // 使用 strtok 分割字符串
    token = strtok(string, delimiters);

    while (token != NULL)
    {
        infoList[count] = token;
        count++;
        token = strtok(NULL, delimiters);
    }
    *num = count;

    return SUCCESS; // 返回0表示成功
}

int pipeControl(char *infoList[], pipeShareData *pipeShareDataSt)
{
    if (strcmp(infoList[0], "PID") == 0)
    {
        for (int i = 1; i < 2; i += 2)
        {
            pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
            pipeShareDataSt->pid_status = 1;
            if (strcmp(infoList[i], "P") == 0)
            {
                pipeShareDataSt->cpid.Kp = atof(infoList[i + 1]);
                LOG(LOG_INFO, "%s PID P = %f\n", __func__, atof(infoList[i + 1]));
            }
            else if (strcmp(infoList[i], "I") == 0)
            {
                pipeShareDataSt->cpid.Ki = atof(infoList[i + 1]);
                LOG(LOG_INFO, "%s PID I = %f\n", __func__, atof(infoList[i + 1]));
            }
            else if (strcmp(infoList[i], "D") == 0)
            {
                pipeShareDataSt->cpid.Kd = atof(infoList[i + 1]);
                LOG(LOG_INFO, "%s PID D = %f\n", __func__, atof(infoList[i + 1]));
            }
            pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);
        }
        return 0;
    }
    if (strcmp(infoList[0], "XMT") == 0)
    {
        for (int i = 1; i < 2; i += 2)
        {
            pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
            if (strcmp(infoList[i], "z") == 0)
            {
                control_xmt_module_infoSt->foundation_zero = atof(infoList[i + 1]);
                LOG(LOG_INFO, "%s control_xmt_module_infoSt->foundation_zero = %f\n", __func__, control_xmt_module_infoSt->foundation_zero);
            }
            else if (strcmp(infoList[i], "h") == 0)
            {
                control_xmt_module_infoSt->hangLenth = atof(infoList[i + 1]);
                LOG(LOG_INFO, "%s control_xmt_module_infoSt->hangLenth = %f\n", __func__, control_xmt_module_infoSt->hangLenth);
            }
            else if (strcmp(infoList[i], "a") == 0)
            {
                control_xmt_module_infoSt->amplify = atof(infoList[i + 1]) / 1000.0;
                LOG(LOG_INFO, "%s control_xmt_module_infoSt->amplify = %f\n", __func__, control_xmt_module_infoSt->amplify );
            }
            pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
        }
        return 0;
    }
    if (strcmp(infoList[0], "N") == 0)
    {
        for (int i = 1; i < 2; i += 2)
        {
           
            if (strcmp(infoList[i], "O") == 0)
            {
                pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
                pipeShareDataSt->amplify_set = atof(infoList[i + 1]);
                pipeShareDataSt->pid_status = 0;
                LOG(LOG_INFO, "%s pipeShareDataSt->amplify_set = %f\n", __func__, pipeShareDataSt->amplify_set);
                pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);
            }
            
            if (strcmp(infoList[i], "A") == 0)
            {
                pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
                control_xmt_module_infoSt->amplify = atof(infoList[i + 1]);
                LOG(LOG_INFO, "%scontrol_xmt_module_infoSt->amplify  = %f\n", __func__, control_xmt_module_infoSt->amplify );
                pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
            }
           
        }
        return 0;
    }
    if (strcmp(infoList[0], "M") == 0) // MOTOR U/D 0/1 length
    {
        for (int i = 1; i < 2; i += 2)
        {
            if (strcmp(infoList[i], "WEIGHT") == 0)
            {
                float tmp  = atof(infoList[i + 1]);
                motorList[0].motor_ctrl.dir = (tmp >= 0) ? 0 : 1;
                motorList[0].motor_ctrl.pul = (int)(abs(tmp)/motorList[0].led*motorList[0].motor_ctrl.micro);
                #ifndef DEBUG_MODE
                ioctl(motor_fd, MOTOR_IOCTL_CMD_SET_VALUE, &motorList[0].motor_ctrl);
                #endif
                LOG(LOG_INFO, "%s motorList WEIGHT = %s\n", __func__, infoList[i + 1] );
                
            }
            else if (strcmp(infoList[i], "LASER") == 0)
            {
                float tmp  = atof(infoList[i + 1]);
                motorList[1].motor_ctrl.dir = (tmp >= 0) ? 0 : 1;
                motorList[1].motor_ctrl.pul = (int)(abs(tmp)/motorList[1].led*motorList[1].motor_ctrl.micro);
                #ifndef DEBUG_MODE
                ioctl(motor_fd, MOTOR_IOCTL_CMD_SET_VALUE, &motorList[1].motor_ctrl);
                #endif
                LOG(LOG_INFO, "%s motorList LASER = %s \n", __func__, infoList[i + 1]);
            }
        }
        return 0;
    }
    if (strcmp(infoList[0], "PSO") == 0)
    {
        pthread_cond_broadcast(&pipeShareDataSt->pso_cond);
        LOG(LOG_INFO, "%s PSO = %s\n", __func__, infoList[0]);
        return 0;
    }
    if (strcmp(infoList[0], "KILL") == 0)
    {
        pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
        pipeShareDataSt->stopThread = 1;
        pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);
        pthread_cond_broadcast(&pipeShareDataSt->stop_cond);
        return 0;
    }

    //现在暂时使用全为 0 和数据作为pid开关
    if (strcmp(infoList[0], "O") == 0)
    {
        return 0;
    }

    if (strcmp(infoList[0], "C") == 0)
    {
        return 0;
    }

    if (strcmp(infoList[0], "Dead") == 0)
    {
        pipeShareDataSt->pid.deadzone = atof(infoList[1]);
        LOG(LOG_INFO, "%s Dead = %f\n", __func__, atof(infoList[1]));
        return 0;
    }

    //Target %f
    if (strcmp(infoList[0], "Target") == 0)
    {
        control_xmt_module_infoSt->target= atof(infoList[1]);
        LOG(LOG_INFO, "%s Target = %f\n", __func__, atof(infoList[1]));
        return 0;
    }

    if (strcmp(infoList[0], "Zero") == 0)
    {
        control_xmt_module_infoSt->foundation_zero= atof(infoList[1]);
        LOG(LOG_INFO, "%s Zero = %f\n", __func__, atof(infoList[1]));
        return 0;
    }

    return 0;
}

void send_buffer(pipeShareData* data, const char *buffer, int len) {
    if (!data) return;
    
    pthread_mutex_lock(&data->write_mutex);
    write(data->send_fd, &len, sizeof(int));
    write(data->send_fd, buffer, len);
    // fsync(data->send_fd);  // 强制刷新缓冲区
    pthread_mutex_unlock(&data->write_mutex);
}

void send_xmt_value_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "XMT %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_cl_value_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "CL %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_pid_error_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "Error %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_pid_intergrate_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "Integrate %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_pid_p_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "P %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_pid_i_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "I %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_pid_d_impl(pipeShareData* data, double value) {
    if (!data) return;
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "D %f\n", value);
    
    send_buffer(data, buffer, len);
}

void send_xmt_command_impl(struct pipeShareData* data, int mode)
{
    if (!data) return;
    
    char buffer[64];
    int len = 0;
    if (mode == 0)
    {
        len = snprintf(buffer, sizeof(buffer), "XMT Error\n");
    }else if (mode == 1)
    {
        len = snprintf(buffer, sizeof(buffer), "XMT Ready\n");
    }
    
    send_buffer(data, buffer, len);
}

void send_cl_command_impl(struct pipeShareData* data, int mode)
{
    if (!data) return;
    
    char buffer[64];
    int len = 0;
    if (mode == 0)
    {
        len = snprintf(buffer, sizeof(buffer), "CL Error\n");
    }else if (mode == 1)
    {
        len = snprintf(buffer, sizeof(buffer), "CL Ready\n");
    }
    
    send_buffer(data, buffer, len);
}

void *pipeDebugThread(void *arg)
{
    char buffer[BUFFER_SIZE];
    char *infoList[MAX_TOKEN];
    int infoNum = 0;
    int *result = malloc(sizeof(int));
    int debug_fd;
    
    if (createPipe(debug_path) < 0)
    {
        *result = ERROR_PIPE_CREATE;
        pthread_exit(result);
    }

        debug_fd = openPipe(debug_path);
        while (pipeShareDataSt->stopThread == 0)
        {
            int len;
            ssize_t  bytes_read= read(debug_fd, buffer, BUFFER_SIZE);
            if (bytes_read == -1)
            {
                break;
            }
            if(buffer[bytes_read-1]  == '\n') buffer[bytes_read-1] = '\0';
            buffer[bytes_read] = '\0';

            LOG(LOG_INFO, "receive %s\n",buffer);

            parseString(buffer, infoList, &infoNum);
            if (infoNum > 0) pipeControl(infoList, pipeShareDataSt);
        }

        close(debug_fd);
        
        LOG(LOG_INFO, "pipeDebugThread exit\n");
        pthread_exit(NULL);  // 线程退出
}


void *pipeReceiveInputThread(void *arg)
{
    pipeShareData *pipeShareDataSt = (pipeShareData *)arg;
    int *result = malloc(sizeof(int));
    int fd = 0;
    char buffer[BUFFER_SIZE];
    char *infoList[MAX_TOKEN];
    int infoNum = 0;

    // 创建接收管道
    if (createPipe(receive_path) < 0)
    {
        *result = ERROR_PIPE_CREATE;
        pthread_exit(result);
    }

    // 创建发送管道
    if (createPipe(send_path) < 0)
    {
        *result = ERROR_PIPE_CREATE;
        pthread_exit(result);
    }

    // 打开管道
    fd = openPipe(receive_path);
    pipeShareDataSt->send_fd = openPipe(send_path);
    motor_fd = open(MOTOR_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        *result = ERROR_PIPE_OPEN;
        pthread_exit(result);
    }

    pthread_mutex_lock(&pipeShareDataSt->start_mutex);
    pthread_cond_signal(&pipeShareDataSt->start_cond);
    pthread_mutex_unlock(&pipeShareDataSt->start_mutex);
    pipeShareDataSt->start_flag = 1;

    LOG(LOG_INFO, "%s start to pipe\n", __func__);

    while (pipeShareDataSt->stopThread == 0)
    {
        int len;
        ssize_t bytes_read = read(fd, &len, sizeof(len));
        bytes_read = read(fd, buffer, len);
        if (bytes_read == -1)
        {
            break;
        }
        if(buffer[bytes_read-1]  == '\n') buffer[bytes_read-1] = '\0';
        buffer[bytes_read] = '\0';

        LOG(LOG_INFO, "receive %s\n",buffer);

        parseString(buffer, infoList, &infoNum);
        if (infoNum > 0) pipeControl(infoList, pipeShareDataSt);
        // LOG(LOG_INFO, "Kp: %f, Ki: %f, Kd: %f\n", pipeShareDataSt->pid.Kp, pipeShareDataSt->pid.Ki, pipeShareDataSt->pid.Kd);
    }

    close(motor_fd);
    close(fd);
    
    LOG(LOG_INFO, "thread exit\n");
    pthread_exit(NULL);  // 线程退出
}