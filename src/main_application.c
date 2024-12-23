#include <stdio.h>

#include "pipe.h"
#include "control_cl_module.h"
#include "control_xmt_module.h"

control_cl_module_info *control_cl_module_infoSt = NULL;
pipeShareData *pipeShareDataSt = NULL;
control_xmt_module_info *control_xmt_module_infoSt = NULL;

int main(int argc, char *argv[])
{
    char path[] = "/var/application";
    int ret = 0;
    pthread_t pipeThread;
    pthread_t clThread;
    pthread_t xmtThread;

    pipeShareDataSt = initPipeShareDataSt();
    control_cl_module_infoSt = initControlClModule();
    control_xmt_module_infoSt = initControlXmtModule();

    pipeShareDataSt->path = path;
    printf("pipeShareDataSt->path = %s\n", pipeShareDataSt->path);

    // 创建线程，传入线程函数和参数（这里我们传入NULL）
    if (pthread_create(&pipeThread, NULL, pipeReceiveInputThread, (void *)pipeShareDataSt) != 0) {
        perror("Thread creation failed");
        return 1;
    }

    if (pthread_create(&clThread, NULL, clReceiveInputThread, (void *)control_cl_module_infoSt) != 0) {
        perror("Thread creation failed");
        return 1;
    }

    if (pthread_create(&xmtThread, NULL, xmtReceiveInputThread, (void *)control_xmt_module_infoSt) != 0) {
        perror("Thread creation failed");
        return 1;
    }

    // 等待线程完成
    if (pthread_join(pipeThread, NULL) != 0) {
        perror("Thread join failed");
        return 1;
    }

    if (pthread_join(clThread, NULL) != 0) {
        perror("Thread join failed");
        return 1;
    }

    if (pthread_join(xmtThread, NULL) != 0) {
        perror("Thread join failed");
        return 1;
    }
    free(control_cl_module_infoSt);
    free(pipeShareDataSt);
    free(control_xmt_module_infoSt);
    printf("Thread has finished execution.\n");

    return ret;
}