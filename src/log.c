#include "log.h"

// 获取当前时间字符串（含微秒）
const char* get_current_time() {
    static char time_str[30];
    struct timeval tv;
    gettimeofday(&tv, NULL);  // 获取时间和微秒
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));
    sprintf(time_str + 19, ".%06ld", tv.tv_usec);  // 追加微秒
    return time_str;
}

