#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>

#define PIPE_NAME "/home/zhouweijie/pipeToQT"  // \u7ba1\u9053\u6587\u4ef6\u8def\u5f84

int main() {
    char buffer[1024];  // \u7528\u4e8e\u5b58\u50a8\u8bfb\u53d6\u5230\u7684\u6570\u636e
    int fd;

    // // \u521b\u5efa\u547d\u540d\u7ba1\u9053\uff08\u5982\u679c\u7ba1\u9053\u4e0d\u5b58\u5728\uff09
    // if (mkfifo(PIPE_NAME, 0666) == -1) {
    //     perror("mkfifo");
    //     exit(EXIT_FAILURE);
    // }

    // \u6253\u5f00\u7ba1\u9053\uff0c\u51c6\u5907\u8bfb\u53d6
    fd = open(PIPE_NAME, O_RDONLY);
    if (fd == -1) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    printf("fd = %d\n", fd);

    // \u6301\u7eed\u8bfb\u53d6\u7ba1\u9053\u4e2d\u7684\u6570\u636e
    while (1) {
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead == -1) {
            perror("read");
            close(fd);
            exit(EXIT_FAILURE);
        } else if (bytesRead == 0) {
            // \u5982\u679c\u6ca1\u6709\u6570\u636e\uff0c\u7ba1\u9053\u88ab\u5173\u95ed
            printf("No more data in the pipe.\n");
            break;
        }

        // \u6253\u5370\u8bfb\u53d6\u5230\u7684\u6570\u636e
        buffer[bytesRead] = '\n';  // \u786e\u4fdd\u5b57\u7b26\u4e32\u4ee5 NULL \u7ed3\u5c3e
        printf("Received from pipe: %s", buffer);
        memset(buffer,0,sizeof(buffer));
    }

    // \u5173\u95ed\u7ba1\u9053
    close(fd);
    return 0;
}