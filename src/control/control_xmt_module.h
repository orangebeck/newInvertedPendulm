#ifndef __CONTROL_XMT_MODULE_H_
#define  __CONTROL_XMT_MODULE_H_

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

#include "xmt_hal_driver.h"
#include "error_codes.h"
#include "pipe.h"
#include "filter.h"
#include "pid_core.h"
#include  "control_cl_module.h"

#define XMT_OFFSET 0.175

#define FILTER_ALPHA 0.01 // 定义低通滤波器参数
#define HOUR 24*5
#define HANG_LENTH 290.0 // 定义悬臂长度154mm
#define AMPLIFY 20.4/1000 // 设定倒摆放大系数
#define QUEUE_LENTH 100
#define QUEUE_LENTH_D 100.0
#define DT 0.25
#define TIME 3600 * (1 / DT) * HOUR

extern pipeShareData *pipeShareDataSt;
extern control_cl_module_info *control_cl_module_infoSt;

typedef struct control_xmt_module_info{
    pthread_mutex_t mutex;      // 互斥锁
    pthread_cond_t cond;        // 条件变量
    float xmtData;            // 共享数据
    int stopThread;
    int fd;

    double foundation_zero;
    double hangLenth;   //mm
    double amplify;
    double dt; //每次采集的时间
    double xmt_zero;
} control_xmt_module_info;

control_xmt_module_info *initControlXmtModule();

void *xmtReceiveInputThread(void *arg);

#endif