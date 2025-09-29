#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "log.h"

#define PIPE_NAME "/home/zhouweijie/pipeToApp"  // \u7ba1\u9053\u8def\u5f84
#define PERIOD 20.0   // \u4e00\u4e2a\u5468\u671f 20 \u79d2

int main() {
    int pipe_fd;
    srand(time(NULL));

    pipe_fd = open(PIPE_NAME, O_RDWR);
    if (pipe_fd == -1) {
        perror("Unable to open the pipe");
        exit(1);
    }

    LOG(LOG_INFO, "pipe_fd = %d\n", pipe_fd);

    LOG(LOG_INFO, "Pipe opened. Sending sine wave data...\n");


    const char *message0 = "XMT Ready\n";
    write(pipe_fd, message0, strlen(message0));
    sleep(3);
    const char *message1 = "CL Ready\n";
    write(pipe_fd, message1, strlen(message1));
    sleep(3);

    int t = 0;  // \u65f6\u95f4\u6b65\uff0c\u5355\u4f4d\u79d2
    while (1) {
        // \u8ba1\u7b97\u6b63\u5f26\u503c\uff08\u8303\u56f4 -1 \u5230 1\uff09
        float value = sin((2.0 * M_PI * t) / PERIOD);

        // \u6784\u9020\u53d1\u9001\u5185\u5bb9\uff0c\u4f8b\u5982\uff1aCL 0.52\n
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "CL %.4f\n", value);
        

        if (write(pipe_fd, buffer, strlen(buffer)) == -1) {
            perror("Write to pipe failed");
            close(pipe_fd);
            exit(1);
        }

        LOG(LOG_INFO, "Written: %s", buffer);
        usleep(1000);
       char buffer1[64];
        snprintf(buffer1, sizeof(buffer1), "XMT %.4f\n", -value*1.5);
        if (write(pipe_fd, buffer1, strlen(buffer1)) == -1) {
            perror("Write to pipe failed");
            close(pipe_fd);
            exit(1);
        }

        LOG(LOG_INFO, "Written: %s", buffer1);

        t++;
        sleep(1);  // \u6bcf\u79d2\u53d1\u9001\u4e00\u6b21
    }

    close(pipe_fd);
    return 0;
}