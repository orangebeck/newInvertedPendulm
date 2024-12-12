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
        res = CL_output_measure_value(buf, value, out1);
        write(info->fd, buf, res);
        res = read(info->fd, buf, 1024);

        CL_ret_data(buf);

        pthread_mutex_lock(&info->mutex);
        info->clData = CL_decode_value(buf);
        pthread_cond_signal(&info->cond);
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

    // 设置定时器间隔为 250 毫秒
    timer_spec.it_value.tv_sec = 0;
    timer_spec.it_value.tv_nsec = SAMPLETIME * 1000000L;  // 250 毫秒
    timer_spec.it_interval.tv_sec = 0;
    timer_spec.it_interval.tv_nsec = SAMPLETIME * 1000000L;  // 250 毫秒

    // 启动定时器
    if (timer_settime(timer_id, 0, &timer_spec, NULL) == -1) {
        perror("timer_settime");
        return NULL;
    }
    printf("CL signal init successfully!\n");
    return timer_id;
}

void *clReceiveInputThread(void *arg)
{
    control_cl_module_info *info = (control_cl_module_info *)arg;
    timer_t timeHandler;
    struct termios termios_tty;
    char *clTtyPath = "/dev/ttySTM1";

    info->fd = fd_init(clTtyPath);
    clConfigTty(info->fd, &termios_tty);

    CL_init(info->fd);

    timeHandler = signal_init(info);

    pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
    pthread_cond_wait(&pipeShareDataSt->stop_cond, &pipeShareDataSt->stop_mutex);
    pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);
    stopThread(info->fd);

    timer_delete(timeHandler);
    pthread_cond_broadcast(&info->cond);
    pthread_exit(NULL);  // 线程退出
}