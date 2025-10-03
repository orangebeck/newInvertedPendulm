#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <sys/time.h>  // 新增头文件
#include <time.h>

typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
} LogLevel;

#define LOG(level, fmt, ...) \
    printf("[%s] [%s]\t" fmt , get_current_time(), \
           (level == LOG_DEBUG) ? "DEBUG" : \
           (level == LOG_INFO) ? "INFO" : \
           (level == LOG_WARNING) ? "WARNING" : "ERROR", ##__VA_ARGS__)

const char* get_current_time();


#endif