#ifndef __CONTROL_CL_MODULE_H__
#define __CONTROL_CL_MODULE_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/msg.h>
#include <sys/ipc.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>

#include "error_codes.h"
#include "cl_hal_driver.h"
#include "pipe.h"
#include "log.h"

#define SAMPLETIME 50.0

extern struct pipeShareData *pipeShareDataSt;

typedef struct control_cl_module_info{
    pthread_mutex_t mutex;      // 互斥锁
    pthread_cond_t cond;        // 条件变量
    double clData;            // 共享数据
    int stopThread;
    int fd;
} control_cl_module_info;

control_cl_module_info *initControlClModule();

void *clReceiveInputThread(void *arg);

#endif