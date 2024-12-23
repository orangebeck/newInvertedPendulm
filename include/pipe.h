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

typedef struct pipeShareData{
    PIDController pid;
    char *path;

    int stopThread;
    pthread_mutex_t stop_mutex; 
    pthread_cond_t stop_cond;
}pipeShareData;

extern control_xmt_module_info *control_xmt_module_infoSt;

pipeShareData *initPipeShareDataSt();


void destroyPipeShareDataSt(pipeShareData *data);

int createPipe(char *path);

int openPipe(char *path);

int parseString(char *string,char *infoList[], int *num);

int pipeControl(char *infoList[], pipeShareData *pipeShareDataSt);

void *pipeReceiveInputThread(void *arg);



#endif