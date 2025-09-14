#include "control_xmt_module.h"

#include "pend_ctrl.h"

void config_tty(int fd, struct termios *termios)
{
    // 清空串口接收缓冲区
    tcflush(fd, TCIOFLUSH);
    // 获取串口参数opt
    tcgetattr(fd, termios);

    // 设置串口输出波特率
    cfsetospeed(termios, B9600);
    // 设置串口输入波特率
    cfsetispeed(termios, B9600);
    // 设置数据位数
    termios->c_cflag &= ~CSIZE;
    termios->c_cflag |= CS8;
    // 校验位
    termios->c_cflag &= ~PARENB;
    termios->c_iflag &= ~INPCK;
    // termios->c_iflag |= IGNCR;  //忽略输入的回车
    //  修改c_iflag来禁用IXON和IXOFF，从而禁用软件流控制 不会出现将0x13自动处理的情况
    termios->c_iflag &= ~(IXON | IXOFF | IXANY|PARMRK);

    termios->c_iflag &= ~ICRNL; // 禁用ICRNL标志位，防止回车符被转换
    // 设置停止位
    termios->c_cflag &= ~CSTOPB;

    termios->c_lflag &= ~ECHO;   // 关闭回显 对面端口不会因为发送什么再收到什么
    termios->c_lflag &= ~ICANON; // 关闭正规模式 这样不会等到0x0A再进行显示 本来是要等一行输入完成之后再回显 非规范模式得到什么输出什么
    termios->c_lflag &= ~IEXTEN;
    termios->c_cc[VSUSP] = _POSIX_VDISABLE;

    // 禁用输出数据的回车符处理
    termios->c_oflag &= ~OPOST; // 当输出0x0A的时候不会在前面自动加上0x0D

    if (tcsetattr(fd, TCSANOW, termios) < 0)
    {
        printf("XMT fail to config tty\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("XMT device set to 9600bps,8N1\n");
    }
}

int xmtfd_init(char *path)
{
    int fd;
    fd = open(path, O_RDWR);
    if (fd < 0)
    {
        printf("XMT fail to open %s\n", path);
    }
    return fd;
}

int config_xmt(int fd, unsigned char *buf, struct xmt_datapacket **xmt_datapacket)
{
    int ret, count = 0;
    *xmt_datapacket = xmt_init();

    xmt_addrinquire(*xmt_datapacket);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    write(fd, buf, ret);

    ret = read(fd, buf, 256);
    if (ret > 0)
    {
    }
    else
    {
        printf("XMT read error");
        return -1;
    }
    printf("xmt clear\n");
    xmt_clear(*xmt_datapacket);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    write(fd, buf, ret);
    printf("xmt ocloop\n");
    xmt_ocloop(*xmt_datapacket, 'C', 0);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    write(fd, buf, ret);
    printf("xmt read displacement\n");
    xmt_read_displacement(*xmt_datapacket, 0, 0);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    write(fd, buf, ret);
     int i = 0;
     printf("send 1     ");
        for(i  =0; i < ret; i++ )
        {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    ret = read(fd, buf, 1024);
    if (ret > 0)
    {
        i = 0;
        printf("receice 1     ");
        for(i  =0; i < ret; i++ )
        {
            printf("%02X ", buf[i]);
        }
        printf("\n");
        xmt_parambuf(buf, *xmt_datapacket);
        xmt_decode_displacement(*xmt_datapacket);
    }
    printf("xmt read displacement\n");
    xmt_read_displacement(*xmt_datapacket, 1, 0);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    i = 0;
     printf("send 2     ");
        for(i  =0; i < ret; i++ )
        {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    write(fd, buf, ret);
    ret = read(fd, buf, 1024);
    if (ret > 0)
    {
                i = 0;
        printf("receice 2     ");
        for(i  =0; i < ret; i++ )
        {
            printf("%02X ", buf[i]);
        }
        printf("\n");
        xmt_parambuf(buf, *xmt_datapacket);
        xmt_decode_displacement(*xmt_datapacket);
    }

    printf("XMT lower is %f, upper is %f\n", (*xmt_datapacket)->lower, (*xmt_datapacket)->upper);

    return 0;
}

control_xmt_module_info *initControlXmtModule()
{
    control_xmt_module_info *info = (control_xmt_module_info *)calloc(1, sizeof(control_xmt_module_info));

    info->amplify = AMPLIFY;
    info->foundation_zero = 0.00;
    info->hangLenth = HANG_LENTH;
    info->xmt_zero = XMT_OFFSET;
    info->dt = (float)SAMPLETIME / 1000.0;

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

void get_file_name(char *buf, size_t bufSize)
{
    time_t rawtime;
    struct tm *timeinfo;
    const char *folder = "data";
    const char *ext = ".csv";

    // 检查 data 文件夹是否存在，不存在则创建
    struct stat st = {0};
    if (stat(folder, &st) == -1) {
        if (mkdir(folder, 0755) == -1) {
            perror("Failed to create directory");
            return;
        }
    }

    // 获取当前时间
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    // 构造文件名路径：data/YYYY-MM-DD_HH-MM-SS.csv
    char timebuf[64];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d_%H-%M-%S", timeinfo);

    // 组合完整路径
    snprintf(buf, bufSize, "%s/%s%s", folder, timebuf, ext);

    // 打印调试信息
    printf("File path: %s\n", buf);
}

FILE* createSaveFile()
{
    FILE *fp;
    char buffer[80];
    get_file_name(buffer,80);

    fp = fopen(buffer, "w");
    if (fp == NULL)
    {
        printf("无法创建文件\n");
        perror("Error opening file");
        return NULL;
    }
    fprintf(fp, "Time,Target,Fundation_Zero,dy,CL,xmt,P,I,D,ff\n");


    return fp;
}

void *PSOControlThread(void *arg)
{   
    control_xmt_module_info *info = (control_xmt_module_info *)arg;
    PSO pso;
    srand(time(NULL));
    while (pipeShareDataSt->stopThread == 0)
    {
        pthread_mutex_lock(&pipeShareDataSt->pso_mutex);
        pthread_cond_wait(&pipeShareDataSt->pso_cond, &pipeShareDataSt->pso_mutex); 
        pthread_mutex_unlock(&pipeShareDataSt->pso_mutex);

        printf("PSOControlThread start\n");
        pso.target = info->foundation_zero;
        pso.ITAETime = 20.0;
        pso.PIDSamplingTime = info->dt;

        backupPID(&pso,  &pipeShareDataSt->pid);
        initPSO(&pso);

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            pso.iteration = i;
            updateParticle(&pso);
            printf("Iteration %d: Best fitness = %f, Kp = %f, Ki = %f, Kd = %f\n", i, pso.globalBestFitness, pso.globalBestPosition[0], pso.globalBestPosition[1], pso.globalBestPosition[2]);
        }
        send_pid_p_impl(pipeShareDataSt,pso.globalBestPosition[0]);
        send_pid_i_impl(pipeShareDataSt,pso.globalBestPosition[1]);
        send_pid_d_impl(pipeShareDataSt,pso.globalBestPosition[2]);

    }
    pthread_exit(NULL);  // 线程退出
}

void *PIDControlThread(void *arg)
{
    control_xmt_module_info *info = (control_xmt_module_info *)arg;
    struct termios termios_tty;
    struct xmt_datapacket *xmt_data_ins;
    char default_path[] = "/dev/ttySTM3";
    unsigned char buf[SENDDATASIZE];
    double xmt_zero = 0.175, PIDresult;
    int res = 0;
    FILE* fp = createSaveFile();
    int count = 0;
    double before_filter;
    cycleBuffer* cb = initCycleBuffer(1000);

    #ifndef DEBUG_MODE
        info->fd = xmtfd_init(default_path);
        config_tty(info->fd, &termios_tty);
        res = config_xmt(info->fd, buf, &xmt_data_ins);
    #endif

    if (res < 0) goto configErr;
    printf("XMT init successfully!\n");
    pipeShareDataSt->send_xmt_command(pipeShareDataSt,1);

    #ifndef DEBUG_MODE
        // xmt_numtodata(xmt_data_ins, 0, 0, XMT_OFFSET);
        // res = xmt_datainlist(xmt_data_ins, buf);
        // int i = 0;
        // for(i  =0; i < res; i++ )
        // {
        //     printf("%02X ", buf[i]);
        // }
        // printf("\n");
        // int debug_ret = write(info->fd, buf, res);
        // printf("XMT set  X = %f mrad! debug_ret = %d\n", XMT_OFFSET, debug_ret);

        PIDresult = XMT_OFFSET;
    #endif

    double tmp =0;


    //临时实验 前馈+PID
    DeviceInfo deviceInfo = {
        .foundation_zero = -0.20, //mm
        .target = -0.20,
        .dt = 0.05,
        .hangLenth = 258,
        .amplify = 14.0,
    };

    CtrlParams ctrlParams;
    Ctrl_GetDefaultParams(0.05, &ctrlParams);

    CtrlState ctrlState;
    Ctrl_Init(&ctrlState);

    pipeShareDataSt->pid_status = 1; //模拟的时候直接进入pid状态


    while (info->stopThread == 0)
    {
        pthread_mutex_lock(&control_cl_module_infoSt->mutex);
        pthread_cond_wait(&control_cl_module_infoSt->cond, &control_cl_module_infoSt->mutex); 
        before_filter = control_cl_module_infoSt->clData;
        pthread_mutex_unlock(&control_cl_module_infoSt->mutex);

        before_filter = firFilterProcess(before_filter);
        
        //如果放大倍数测量的时候就设置 pid_status == 0 ， 向xmt发送的消息为设定放大值
        if(pipeShareDataSt->pid_status == 0)
        {
            PIDresult = pipeShareDataSt->amplify_set;
        }else if (pipeShareDataSt->pid_status == 1)
        {
            ctrlParams.Kp = pipeShareDataSt->pid.Kp;
            ctrlParams.Ki = pipeShareDataSt->pid.Ki;
            ctrlParams.Kd = pipeShareDataSt->pid.Kd;
            deviceInfo.target = control_xmt_module_infoSt->target;

            PIDresult = Controller_Step(before_filter, &deviceInfo, &ctrlParams, &ctrlState, XMT_OFFSET, -1); 

            // pipeShareDataSt->send_pid_error(pipeShareDataSt, pipeShareDataSt->pid.prevError);
            // pipeShareDataSt->send_pid_intergrate(pipeShareDataSt, pipeShareDataSt->pid.integral);
            // pipeShareDataSt->send_xmt_value(pipeShareDataSt, PIDresult);
        }
        printf("PIDresult=%f\n",PIDresult);
        #ifndef DEBUG_MODE
        xmt_numtodata(xmt_data_ins, 0, 0, PIDresult);
        res = xmt_datainlist(xmt_data_ins, buf);
        // printf("xmt send data = %f \n", PIDresult);
        write(info->fd, buf, res);
        #endif

        pipeShareDataSt->send_xmt_value(pipeShareDataSt, PIDresult);

        fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",   (double)count * SAMPLETIME/1000.0, 
                                                                control_xmt_module_infoSt->target, 
                                                                deviceInfo.foundation_zero,
                                                                ctrlState.dy_f,
                                                                before_filter, 
                                                                PIDresult, 
                                                                ctrlState.last_P, 
                                                                ctrlState.last_I, 
                                                                ctrlState.last_D, 
                                                                ctrlState.last_FF);
        
        if (count % 100000 == 0 && count != 0)
        {
            fclose(fp);
            fp = createSaveFile();
            if (fp == NULL)
            {
                printf("无法创建文件\n");
                return NULL;
            }
        }
        count += 1;
    }
#ifndef DEBUG_MODE
    free(xmt_data_ins);
#endif
    close(info->fd);

configErr:
    pthread_exit(NULL);  // 线程退出
}

void *xmtReceiveInputThread(void *arg)
{
    pthread_t PSOThread;
    pthread_t PIDThread;

    if (pthread_create(&PSOThread, NULL, PSOControlThread, (void *)arg) != 0) {
        perror("Thread creation failed");
        return NULL;
    }

    if (pthread_create(&PIDThread, NULL, PIDControlThread, (void *)arg) != 0) {
        perror("Thread creation failed");
        return NULL;
    }

    if (pthread_join(PSOThread, NULL) != 0) {
    perror("Thread join failed");
    return NULL;
    }

    if (pthread_join(PIDThread, NULL) != 0) {
        perror("Thread join failed");
        return NULL;
    }
    return NULL;
}