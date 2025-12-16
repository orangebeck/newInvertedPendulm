#include "control_cl_module.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SERVER_IP "192.168.3.80"
#define SERVER_PORT 24685
#define BUFFER_SIZE 1024

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
        LOG(LOG_ERROR, "CL fail to config tty\n");
        return ERROR_IO_FAILURE;
    }
    else
    {
        LOG(LOG_INFO, "CL device set to 115200bps,8N1\n");
        return SUCCESS;
    }
}

int  connect_CL_Socket()
{
    struct sockaddr_in server_addr;
    ssize_t bytes_sent, bytes_received;

    // 创建TCP socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("invalid address");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Connecting to %s:%d...\n", SERVER_IP, SERVER_PORT);

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("connection failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Connected successfully!\n");
    return sockfd;
}

void send_data(int sockfd, unsigned char* data, int size)
{
    ssize_t   bytes_sent = send(sockfd, data, size, 0);
    if (bytes_sent < 0) {
        perror("send failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
}

int receive_data(int sockfd, unsigned char* buffer, int size)
{
        // 接收返回报文
    // ssize_t bytes_received = recv(sockfd, buffer, size, 0);
    int bytes_received_int = (int)recv(sockfd, buffer, size, 0);
    if (bytes_received_int < 0) {
        perror("receive failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (bytes_received_int == 0) {
        printf("Connection closed by server\n");
    }

    return bytes_received_int;
}

int fd_init(char *path)
{
    int fd;
    fd = open(path, O_RDWR);
    if (fd < 0)
    {
        LOG(LOG_ERROR, "CL fail to open %s\n", path);
    }
    return fd;
}


void CL_init(int fd)
{
    int res;
    char buf[1024];
    res = CL_enter_measurement_mode(buf);
    send_data(fd, buf, res);
    res = receive_data(fd, buf, 1024);
    CL_ret_data(buf);

    res = CL_stop_data_storage(buf);
    send_data(fd, buf, res);
    res = receive_data(fd, buf, 1024);
    CL_ret_data(buf);

    res = CL_init_data_storage(buf);
    send_data(fd, buf, res);
    res = receive_data(fd, buf, 1024);
    CL_ret_data(buf);

    res = CL_start_data_storage(buf);
    send_data(fd, buf, res);
    res = receive_data(fd, buf, 1024);
    CL_ret_data(buf);
}

void timer_thread(int signo, siginfo_t *sigInfo, void *context)
{   
    if (signo == SIGALRM) {
    
        static float res = 0;
        char buf[1024];

        control_cl_module_info *info = (control_cl_module_info *)sigInfo->si_value.sival_ptr;

        #ifndef DEBUG_MODE
            res = CL_output_measure_value(buf, value, out1);
            send_data(info->fd, buf, res);
            res = receive_data(info->fd, buf, 1024);
            CL_ret_data(buf);
        #endif

        pthread_mutex_lock(&info->mutex);
        #ifndef DEBUG_MODE
            info->clData = CL_decode_value(buf);
        #else
            info->clData = (double)(rand() % 1000) / 100000.0;
        #endif
        pipeShareDataSt->send_cl_value(pipeShareDataSt, info->clData);

        // pthread_cond_signal(&info->cond);
        pthread_cond_broadcast(&info->cond); //多个线程都会等待

        pthread_mutex_unlock(&info->mutex);
    }else
    {
        LOG(LOG_INFO, "timer_thread\n");
    }
}

void stopThread(int fd)
{
    int res;
    char buf[1024];
    res = CL_stop_data_storage(buf);
    send_data(fd, buf, res);
    res = receive_data(fd, buf, 1024);
    CL_ret_data(buf);
    LOG(LOG_INFO, "Close CL thread\n");
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
    LOG(LOG_INFO, "CL signal init successfully!\n");
    pipeShareDataSt->send_cl_command(pipeShareDataSt, 1);
    return timer_id;
}

void *clReceiveInputThread(void *arg)
{
    char thread_name[16];
    snprintf(thread_name, sizeof(thread_name), "%s", "clReceive");
    pthread_setname_np(pthread_self(), thread_name);

    control_cl_module_info *info = (control_cl_module_info *)arg;
    timer_t timeHandler;
    struct termios termios_tty;

    #ifndef DEBUG_MODE
    // char *clTtyPath = "/dev/ttySTM1";
    // info->fd = fd_init(clTtyPath);
    // clConfigTty(info->fd, &termios_tty);
    info->fd=connect_CL_Socket();
    CL_init(info->fd);
    #endif
    
    timeHandler = signal_init(info);

    pthread_mutex_lock(&pipeShareDataSt->stop_mutex);
    pthread_cond_wait(&pipeShareDataSt->stop_cond, &pipeShareDataSt->stop_mutex);
    pthread_mutex_unlock(&pipeShareDataSt->stop_mutex);

    #ifndef DEBUG_MODE
    stopThread(info->fd);
    #endif

    timer_delete(timeHandler);
    pthread_cond_broadcast(&info->cond);
    LOG(LOG_INFO, "%s thread exit\n", __func__);
    pthread_exit(NULL);  // 线程退出
}