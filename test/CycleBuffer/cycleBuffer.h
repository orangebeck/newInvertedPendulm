#ifndef CYCLEBUFFER_H
#define CYCLEBUFFER_H

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>

typedef struct {
    double *buffer;     // 缓冲区指针
    int capacity;    // 缓冲区总容量
    int head;        // 头部索引（最新数据位置）
    int tail;        // 尾部索引（最旧数据位置）
    int count;       // 当前有效数据量
    double sum;         // 数据总和（用于快速计算平均值）
    double min;         // 当前最小值
    double max;         // 当前最大值
} CycleBuffer;

CycleBuffer* cycle_buffer_init(double capacity);

void cycle_buffer_free(CycleBuffer *cb);

bool cycle_buffer_resize(CycleBuffer *cb, int new_capacity);

void cycle_buffer_push(CycleBuffer *cb, double value);

double cycle_buffer_pop(CycleBuffer *cb);

int cycle_buffer_count(const CycleBuffer *cb) ;

double cycle_buffer_avg(const CycleBuffer *cb);

double cycle_buffer_min(const CycleBuffer *cb);

double cycle_buffer_max(const CycleBuffer *cb);

void cycle_buffer_print(const CycleBuffer *cb);



#endif