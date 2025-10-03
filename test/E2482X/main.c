#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>
#include "log.h"

#define SERVER_IP "192.168.1.250"
#define SERVER_PORT 6000
#define BUFFER_SIZE 1500

int sockfd;
unsigned char buffer[BUFFER_SIZE];
int count = 0;

// 要发送的报文数据
unsigned char send_data_ping[] = {
    0xAA, 0x55, 0x08, 0xD1, 0xA3, 0x01, 0x55, 0xAA
};

// 要发送的报文数据
unsigned char send_data_sample[] = {
    0xAA, 0x55, 0x0C, 0x15, 0xA1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x55, 0xAA
};

// 要发送的报文数据
unsigned char send_data_sample_rate[] = {
    0xAA, 0x55, 0x09, 0xD0, 0xA3, 0x01, 0x01, 0x55, 0xAA
};

void send_data(int sockfd, unsigned char* data, int size)
{
    ssize_t   bytes_sent = send(sockfd, data, size, 0);
    if (bytes_sent < 0) {
        perror("send failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // printf("Sent %zd bytes\n ", bytes_sent);
}

void receive_data(int sockfd, unsigned char* buffer, int size)
{
        // 接收返回报文
    ssize_t bytes_received = recv(sockfd, buffer, size, 0);
    if (bytes_received < 0) {
        perror("receive failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (bytes_received == 0) {
        printf("Connection closed by server\n");
    } else {
        // printf("Received %zd bytes:\n", bytes_received);
    }
}

// 定时器回调函数
void timer_handler(union sigval val) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    // 这里添加你的处理逻辑
    LOG(LOG_DEBUG, "start  time\n");

    send_data(sockfd, send_data_sample, sizeof(send_data_sample));

    receive_data(sockfd, buffer, BUFFER_SIZE);

    int index = 16;
    double sum = 0;
    for(int i = 0; i < 480; i++)
    {
        // 推荐的简洁写法
        int tmp = (buffer[index + i * 3] << 16) | (buffer[index + 1 + i * 3] << 8) |  buffer[index + 2 + i * 3];
        double value = tmp & 0x800000 ? (double)(0x800000 -tmp) / 0x800000 : (double)tmp / 0x800000;
        sum += value;
        if(i == 120)
        {
            LOG(LOG_DEBUG, "120  time %f\n", sum / 120.0 * 0.04);
        }else if(i==240)
        {
            LOG(LOG_DEBUG, "240  time  %f\n", sum / 240.0 * 0.04);
        }
    }
    sum = sum / 480.0 * 0.04;
    LOG(LOG_DEBUG, "end  time\n");
    printf("%f,%f\n", count++ *0.05, sum);
}

int main() {
    timer_t timer_id;
    struct sigevent sev;
    struct itimerspec its;
    
    // 设置定时器事件
    sev.sigev_notify = SIGEV_THREAD;  // 创建新线程调用处理函数
    sev.sigev_value.sival_ptr = &timer_id;
    sev.sigev_notify_function = timer_handler;
    sev.sigev_notify_attributes = NULL;
    
    // 创建定时器
    if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id) == -1) {
        perror("timer_create");
        exit(EXIT_FAILURE);
    }
    
    // 设置定时器参数：50ms间隔
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 50000000;  // 50ms
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 50000000;  // 50ms
    
    struct sockaddr_in server_addr;
    ssize_t bytes_sent, bytes_received;

    // 创建TCP socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 设置服务器地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("invalid address");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Connecting to %s:%d...\n", SERVER_IP, SERVER_PORT);

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("connection failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Connected successfully!\n");

    // 发送报文
    send_data(sockfd, send_data_ping, sizeof(send_data_ping));

    // 接收返回报文
    receive_data(sockfd, buffer, BUFFER_SIZE);
    
    // send_data(sockfd, send_data_sample_rate, sizeof(send_data_sample_rate));

// 启动定时器
    if (timer_settime(timer_id, 0, &its, NULL) == -1) {
        perror("timer_settime");
        exit(EXIT_FAILURE);
    }
    
    printf("POSIX 50ms timer started...\n");
    
    // 主线程保持运行
    while (1) {
        sleep(1);
    }
    
    // 清理（通常不会执行到这里）
    timer_delete(timer_id);
    // 关闭连接
    close(sockfd);
    printf("Connection closed.\n");

    return 0;
}