#include "pipe.h"
#include "motor.h"

int  stopThreadFlag = 0;

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
            .enx = 0,
            .micro = 400,
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

    PID_Init(&data->pid, 0, 0, 0, -0.005, 0.005);

    if (pthread_mutex_init(&data->stop_mutex, NULL) != 0) {
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
        printf("Named pipe created at %s\n", path);
        return SUCCESS;
    }
    else
    {
        printf("Named pipe already exists at %s\n", path);
        return SUCCESS;
    }
}

int openPipe(char *path)
{
    int fd = open(path, O_RDWR);
    if (fd == -1)
    {
        perror("open");
        return ERROR_PIPE_OPEN;
    }
    printf("Named pipe opened at %s\n", path);
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
            if (strcmp(infoList[i], "P") == 0)
            {
                pipeShareDataSt->pid.Kp = atof(infoList[i + 1]);
            }
            else if (strcmp(infoList[i], "I") == 0)
            {
                pipeShareDataSt->pid.Ki = atof(infoList[i + 1]);
            }
            else if (strcmp(infoList[i], "D") == 0)
            {
                pipeShareDataSt->pid.Kd = atof(infoList[i + 1]);
            }
            pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);
        }
    }
    if (strcmp(infoList[0], "XMT") == 0)
    {
        for (int i = 1; i < 2; i += 2)
        {
            pthread_mutex_lock(&control_xmt_module_infoSt->mutex);
            if (strcmp(infoList[i], "z") == 0)
            {
                control_xmt_module_infoSt->foundation_zero = atof(infoList[i + 1]);
            }
            else if (strcmp(infoList[i], "h") == 0)
            {
                control_xmt_module_infoSt->hangLenth = atof(infoList[i + 1]);
            }
            else if (strcmp(infoList[i], "a") == 0)
            {
                control_xmt_module_infoSt->amplify = atof(infoList[i + 1]);
            }
            pthread_mutex_unlock(&control_xmt_module_infoSt->mutex);
        }
    }
    if (strcmp(infoList[0], "MOTOR") == 0) // MOTOR U/D 0/1 length
    {
        for (int i = 1; i < 2; i += 2)
        {
            if (strcmp(infoList[i], "U") == 0)
            {
                motorList[atoi(infoList[i + 1])].motor_ctrl.dir = 0;
                motorList[atoi(infoList[i + 1])].motor_ctrl.pul = (int)(atof(infoList[i + 2])/motorList[atoi(infoList[i + 1])].led);
                ioctl(motor_fd, MOTOR_IOCTL_CMD_SET_VALUE, &motorList[atoi(infoList[i + 1])].motor_ctrl);
            }
            else if (strcmp(infoList[i], "D") == 0)
            {
                motorList[atoi(infoList[i + 1])].motor_ctrl.dir = 1;
                motorList[atoi(infoList[i + 1])].motor_ctrl.pul = (int)(atof(infoList[i + 2])/motorList[atoi(infoList[i + 1])].led);
                ioctl(motor_fd, MOTOR_IOCTL_CMD_SET_VALUE, &motorList[atoi(infoList[i + 1])].motor_ctrl);
            }
        }
    }
    if (strcmp(infoList[0], "KILL") == 0)
    {
        pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
        pipeShareDataSt->stopThread = 1;
        pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);
        pthread_cond_broadcast(&pipeShareDataSt->stop_cond);
    }

    return 0;
}

void *pipeReceiveInputThread(void *arg)
{
    pipeShareData *pipeShareDataSt = (pipeShareData *)arg;
    int *result = malloc(sizeof(int));
    int fd = 0;
    char buffer[BUFFER_SIZE];
    char *infoList[MAX_TOKEN];
    int infoNum = 0;

    printf("path = %s\n", pipeShareDataSt->path);

    // 创建管道
    if (createPipe(pipeShareDataSt->path) < 0)
    {
        *result = ERROR_PIPE_CREATE;
        pthread_exit(result);
    }

    // 打开管道
    fd = openPipe(pipeShareDataSt->path);
    motor_fd = open(MOTOR_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        *result = ERROR_PIPE_OPEN;
        pthread_exit(result);
    }

    while (pipeShareDataSt->stopThread == 0)
    {
        ssize_t bytes_read = read(fd, buffer, BUFFER_SIZE - 1);
        if (bytes_read == -1)
        {
            break;
        }
        if(buffer[bytes_read-1]  == '\n') buffer[bytes_read-1] = '\0';
        buffer[bytes_read] = '\0';

        printf("receive %s\n",buffer);

        parseString(buffer, infoList, &infoNum);
        if (infoNum > 0) pipeControl(infoList, pipeShareDataSt);
        printf("Kp: %f, Ki: %f, Kd: %f\n", pipeShareDataSt->pid.Kp, pipeShareDataSt->pid.Ki, pipeShareDataSt->pid.Kd);
    }

    close(motor_fd);
    close(fd);
    
    printf("thread exit\n");
    pthread_exit(NULL);  // 线程退出
}