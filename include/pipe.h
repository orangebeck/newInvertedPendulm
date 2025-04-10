#ifndef __PIPE_H__
#define __PIPE_H__

#include "error_codes.h"
#include "pid_core.h"
#include "control_cl_module.h"
#include "StControl_xmt_module_info.h"

#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#define BUFFER_SIZE 256
#define MAX_TOKEN 100

struct pipeShareData;

// 函数指针类型定义
typedef void (*send_func_value_t)(struct pipeShareData*, double);
typedef void (*send_func_command_t)(struct pipeShareData*, int);

typedef struct pipeShareData{
    PIDController pid;
    char *path;

    int stopThread;
    pthread_mutex_t stop_mutex; 
    pthread_cond_t stop_cond;

    pthread_mutex_t pso_mutex; 
    pthread_cond_t pso_cond;

    pthread_mutex_t start_mutex; 
    pthread_cond_t start_cond;
    int start_flag;

    //设置一个互斥锁
    int send_fd;
    pthread_mutex_t write_mutex;

    // 函数指针
    send_func_value_t send_xmt_value;
    send_func_value_t send_cl_value;
    send_func_value_t send_pid_error;
    send_func_value_t send_pid_intergrate;

    send_func_command_t send_xmt_command;
    send_func_command_t send_cl_command;
}pipeShareData;

extern control_xmt_module_info *control_xmt_module_infoSt;

pipeShareData *initPipeShareDataSt();


void destroyPipeShareDataSt(pipeShareData *data);

int createPipe(char *path);

int openPipe(char *path);

int parseString(char *string,char *infoList[], int *num);

int pipeControl(char *infoList[], pipeShareData *pipeShareDataSt);

void *pipeReceiveInputThread(void *arg);

void send_buffer(pipeShareData* data, const char *buffer, int len);

void send_xmt_value_impl(pipeShareData* data, double value);

void send_cl_value_impl(pipeShareData* data, double value);

void send_pid_error_impl(pipeShareData* data, double value);

void send_pid_intergrate_impl(pipeShareData* data, double value);

void send_xmt_command_impl(struct pipeShareData* data, int mode);

void send_cl_command_impl(struct pipeShareData* data, int mode);

#endif