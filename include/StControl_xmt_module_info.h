#ifndef __STCONTROL_XMT_MODULE_INFO_H_
#define  __STCONTROL_XMT_MODULE_INFO_H_
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/msg.h>
#include <sys/ipc.h>
#include <sys/wait.h>
#include <signal.h>


typedef struct control_xmt_module_info{
    pthread_mutex_t mutex;      // 互斥锁
    pthread_cond_t cond;        // 条件变量
    float xmtData;            // 共享数据
    int stopThread;
    int fd;

    double foundation_zero; //0.15mrad对应的测量值
    double hangLenth;   //mm
    double amplify;
    double dt; //每次采集的时间
    double xmt_zero;
    double target;   //设定目标值
} control_xmt_module_info;

#endif