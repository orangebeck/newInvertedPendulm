#include "control_cl_module.h"

control_cl_module_info *initControlClModule()
{
    control_cl_module_info *info = (control_cl_module_info *)calloc(1, sizeof(control_cl_module_info));

    if (pthread_mutex_init(&info->mutex, NULL) != 0) {
        perror("Mutex initialization failed");
        free(info);
        return NULL;
    }

    if (pthread_cond_init(&info->cond, NULL) != 0) {
        perror("Condition variable initialization failed");
        pthread_mutex_destroy(&info->mutex);
        free(info);
        return NULL;
    }

    return info;
}

int clConfigTty(int fd, struct termios *termios)
{
    // 清空串口接收缓冲区
    tcflush(fd, TCIOFLUSH);
    // 获取串口参数opt
    tcgetattr(fd, termios);

    // 设置串口输出波特率
    cfsetospeed(termios, B115200);
    // 设置串口输入波特率
    cfsetispeed(termios, B115200);
    // 设置数据位数
    termios->c_cflag &= ~CSIZE;
    termios->c_cflag |= CS8;
    // 校验位
    termios->c_cflag &= ~PARENB;
    termios->c_iflag &= ~INPCK;
    // 修改c_iflag来禁用IXON和IXOFF，从而禁用软件流控制 不会出现将0x13自动处理的情况
    termios->c_iflag &= ~(IXON | IXOFF | IXANY);

    // 设置停止位
    termios->c_cflag &= ~CSTOPB;

    termios->c_lflag &= ~ECHO; // 关闭回显 对面端口不会因为发送什么再收到什么

    // 禁用输出数据的回车符处理
    termios->c_oflag &= ~OPOST; // 当输出0x0A的时候不会在前面自动加上0x0D

    if (tcsetattr(fd, TCSANOW, termios) < 0)
    {
        printf("CL fail to config tty\n");
        return ERROR_IO_FAILURE;
    }
    else
    {
        printf("CL device set to 115200bps,8N1\n");
        return SUCCESS;
    }
}

int fd_init(char *path)
{
    int fd;
    fd = open(path, O_RDWR);
    if (fd < 0)
    {
        printf("CL fail to open %s\n", path);
    }
    return fd;
}

void CL_init(int fd)
{
    int res;
    char buf[1024];
    res = CL_enter_measurement_mode(buf);
    write(fd, buf, res);
    res = read(fd, buf, 1024);
    CL_ret_data(buf);

    res = CL_stop_data_storage(buf);
    write(fd, buf, res);
    res = read(fd, buf, 1024);
    CL_ret_data(buf);

    res = CL_init_data_storage(buf);
    write(fd, buf, res);
    res = read(fd, buf, 1024);
    CL_ret_data(buf);

    res = CL_start_data_storage(buf);
    write(fd, buf, res);
    res = read(fd, buf, 1024);
    CL_ret_data(buf);
}

void timer_thread(int signo, siginfo_t *sigInfo, void *context)
{   
    if (signo == SIGALRM) {
    
        static float res = 0;
        char buf[1024];

        control_cl_module_info *info = (control_cl_module_info *)sigInfo->si_value.sival_ptr;

        #ifndef DEBUG_MODE
            int32_t values[8] = {0};
            size_t read_len = 0;
            err = capaNCDT_read_measurements(&data_handle, values, &read_len);
            if (err == CAPA_OK) {
                printf("Channel %d: %d ", 1, values[1]);
                printf("distance %.4f\n", (values[1] / 16777215.0f) * 500.0f);
            } else {
                printf("Failed to read measurements: %d\n", err);
            }
        #endif

        pthread_mutex_lock(&info->mutex);
        #ifndef DEBUG_MODE
            info->clData = (values[1] / 16777215.0f) * 500.0f /1000.0f;
        #else
            info->clData = (double)(rand() % 1000) / 100000.0;
        #endif
        pipeShareDataSt->send_cl_value(pipeShareDataSt, info->clData);

        // pthread_cond_signal(&info->cond);
        pthread_cond_broadcast(&info->cond); //多个线程都会等待

        pthread_mutex_unlock(&info->mutex);
    }else
    {
        printf("timer_thread\n");
    }
}

void stopThread(int fd)
{
    int res;
    char buf[1024];
    res = CL_stop_data_storage(buf);
    write(fd, buf, res);
    res = read(fd, buf, 1024);
    CL_ret_data(buf);
    printf("Close CL thread\n");
    close(fd);
}

timer_t signal_init(control_cl_module_info *info)
{
    struct sigevent sev;
    struct itimerspec timer_spec;
    timer_t timer_id;

    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;   // 启用 siginfo_t 以接收附加数据
    sa.sa_sigaction = timer_thread;
    if (sigaction(SIGALRM, &sa, NULL) == -1) {
        perror("sigaction");
        return NULL;
    }


    // 设置 sigevent 结构体，使用 SIGEV_SIGNAL 通知方式
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = (void *)info;   // 传递句柄（或者其他上下文数据）

    // 创建定时器
    if (timer_create(CLOCK_REALTIME, &sev, &timer_id) == -1) {
        perror("timer_create");
        return NULL;
    }

    // 设置定时器间隔为 SAMPLETIME 毫秒
    timer_spec.it_value.tv_sec = 0;
    timer_spec.it_value.tv_nsec = SAMPLETIME * 1000000L;  // 250 毫秒
    timer_spec.it_interval.tv_sec = 0;
    timer_spec.it_interval.tv_nsec = SAMPLETIME * 1000000L;  // 250 毫秒

    if(pipeShareDataSt->start_flag == 0)
    {
        pthread_mutex_lock(&pipeShareDataSt->start_mutex);
        pthread_cond_wait(&pipeShareDataSt->start_cond, &pipeShareDataSt->start_mutex);
        pthread_mutex_unlock(&pipeShareDataSt->start_mutex);
    }


    // 启动定时器
    if (timer_settime(timer_id, 0, &timer_spec, NULL) == -1) {
        perror("timer_settime");
        return NULL;
    }
    printf("CL signal init successfully!\n");
    pipeShareDataSt->send_cl_command(pipeShareDataSt, 1);
    return timer_id;
}


void InitCapaNCDT()
{
    // 设置控制器信息
    const char *sensor_ip = "192.168.1.150";
    uint16_t ctrl_port = 23;
    uint16_t data_port = 10001;

    printf("Connecting to controller...\n");
    err = capaNCDT_connect(&ctrl_handle, sensor_ip, ctrl_port);
    if (err != CAPA_OK) {
        printf("Failed to connect to controller: %d\n", err);
        return 1;
    }

    capaNCDT_get_channel_info(&ctrl_handle, 1, &channel_info[0]);
    printf("Channel 1 Info:\n");
    printf("  Article No: %s\n", channel_info[0].article_no);
    printf("  Name: %s\n", channel_info[0].name);
    printf("  Serial No: %s\n", channel_info[0].serial_no);
    printf("  Offset: %.2f\n", channel_info[0].offset);
    printf("  Range: %.2f\n", channel_info[0].range);
    printf("  Unit: %s\n", channel_info[0].unit);
    printf("  Data Type: %d\n", channel_info[0].data_type);

    capaNCDT_set_trigger_mode(&ctrl_handle, CAPA_TRIGGER_CONTINUOUS);



    printf("Connecting to data port...\n");
    err = capaNCDT_connect(&data_handle, sensor_ip, data_port);
    if (err != CAPA_OK) {
        printf("Failed to connect to data port: %d\n", err);
        capaNCDT_disconnect(&ctrl_handle);
        return 1;
    }
}

void *clReceiveInputThread(void *arg)
{
    control_cl_module_info *info = (control_cl_module_info *)arg;
    timer_t timeHandler;
    struct termios termios_tty;

    // #ifndef DEBUG_MODE
    // char *clTtyPath = "/dev/ttySTM1";
    // info->fd = fd_init(clTtyPath);
    // clConfigTty(info->fd, &termios_tty);
    // CL_init(info->fd);
    // #endif

    InitCapaNCDT();

    
    timeHandler = signal_init(info);

    pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
    pthread_cond_wait(&pipeShareDataSt->stop_cond, &pipeShareDataSt->stop_mutex);
    pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);

    #ifndef DEBUG_MODE
    // stopThread(info->fd);

    capaNCDT_disconnect(&data_handle);
    capaNCDT_disconnect(&ctrl_handle);
    #endif

    timer_delete(timeHandler);
    pthread_cond_broadcast(&info->cond);
    pthread_exit(NULL);  // 线程退出
}