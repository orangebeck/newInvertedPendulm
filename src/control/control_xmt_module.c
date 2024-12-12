#include "control_xmt_module.h"

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
    printf("xmt readdisplacement\n");
    xmt_read_displacement(*xmt_datapacket, 0, 0);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    write(fd, buf, ret);
    ret = read(fd, buf, 1024);
    if (ret > 0)
    {
        xmt_parambuf(buf, *xmt_datapacket);
        xmt_decode_displacement(*xmt_datapacket);
    }
    printf("xmt readdisplacement\n");
    xmt_read_displacement(*xmt_datapacket, 1, 0);
    ret = xmt_datainlist(*xmt_datapacket, buf);
    write(fd, buf, ret);
    ret = read(fd, buf, 1024);
    if (ret > 0)
    {
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
    info->foundation_zero = 0;
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
    char buffer_type[] = ".csv";

    // 获取当前的系统时间
    time(&rawtime);
    // 转换为本地时间
    timeinfo = localtime(&rawtime);

    // 格式化时间。例如：2023-03-27 15:00:00
    // Ensure the buffer is large enough to accommodate the date-time format plus the file extension.
    strftime(buf, bufSize - strlen(buffer_type), "%Y-%m-%d %H:%M:%S", timeinfo);
    printf("%s\n", buf);

    // Append the file type
    strcat(buf, buffer_type);
    printf("%s\n", buf);
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
        return NULL;
    }
    fprintf(fp, "count,CL_origin,CL_filter,xmt_control,zero\n");


    return fp;
}

void *xmtReceiveInputThread(void *arg)
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

    info->fd = xmtfd_init(default_path);
    config_tty(info->fd, &termios_tty);

    res = config_xmt(info->fd, buf, &xmt_data_ins);
    if (res < 0) goto configErr;
    printf("XMT init successfully!\n");

    xmt_numtodata(xmt_data_ins, 0, 0, XMT_OFFSET);
    res = xmt_datainlist(xmt_data_ins, buf);
    write(info->fd, buf, res);

    while (info->stopThread == 0)
    {
        pthread_mutex_lock(&control_cl_module_infoSt->mutex);
        pthread_cond_wait(&control_cl_module_infoSt->cond, &control_cl_module_infoSt->mutex); 
        before_filter = control_cl_module_infoSt->clData;
        pthread_mutex_unlock(&control_cl_module_infoSt->mutex);

        before_filter = firFilterProcess(before_filter);
        pthread_mutex_lock(&info->mutex);
        PIDresult = -PID_Compute(&pipeShareDataSt->pid, info->foundation_zero / info->hangLenth / info->amplify, before_filter / info->hangLenth / info->amplify, info->dt);
        printf("PID_Compute = %f\n", PIDresult);
        pthread_mutex_unlock(&info->mutex);
        PIDresult += info->xmt_zero;
        info->xmt_zero = PIDresult;
        
        printf("PIDresult = %f\n", PIDresult);

        xmt_numtodata(xmt_data_ins, 0, 0, PIDresult);
        res = xmt_datainlist(xmt_data_ins, buf);
        write(info->fd, buf, res);

         fprintf(fp, "%d,%lf,%lf,%lf,%lf\n", count, before_filter, info->xmt_zero, ( info->foundation_zero)/ info->hangLenth / info->amplify);
        if (count % 100000 == 0 && count != 0)
        {
            fclose(fp);
            fp = createSaveFile();
            if (fp == NULL)
            {
                printf("无法创建文件\n");
                return NULL;
            }
            fprintf(fp, "count,CL_origin,CL_filter,xmt_control\n");
        }
        count += 1;
    }
    
    free(xmt_data_ins);
    close(info->fd);
configErr:
    pthread_exit(NULL);  // 线程退出
}