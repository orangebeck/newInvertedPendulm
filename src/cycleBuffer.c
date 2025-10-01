#include "cycleBuffer.h"


// 初始化缓冲区
CycleBuffer* cycle_buffer_init(double capacity) {
    if (capacity <= 0) return NULL;

    CycleBuffer *cb = (CycleBuffer*)malloc(sizeof(CycleBuffer));
    if (!cb) return NULL;

    cb->buffer = (double*)malloc(sizeof(double) * capacity);
    if (!cb->buffer) {
        free(cb);
        return NULL;
    }

    cb->capacity = capacity;
    cb->head = cb->tail = cb->count = 0;
    cb->sum = 0;
    cb->min = INT_MAX;
    cb->max = INT_MIN;
    return cb;
}

// 释放缓冲区
void cycle_buffer_free(CycleBuffer *cb) {
    if (cb) {
        free(cb->buffer);
        free(cb);
    }
}

// 动态调整缓冲区大小
bool cycle_buffer_resize(CycleBuffer *cb, int new_capacity) {
    if (!cb || new_capacity <= 0) return false;

    double *new_buffer = (double*)malloc(sizeof(double) * new_capacity);
    if (!new_buffer) return false;

    // 复制旧数据到新缓冲区（保持顺序）
    int i = 0;
    while (cb->count > 0 && i < new_capacity) {
        new_buffer[i++] = cb->buffer[cb->tail];
        cb->tail = (cb->tail + 1) % cb->capacity;
        cb->count--;
    }

    free(cb->buffer);
    cb->buffer = new_buffer;
    cb->capacity = new_capacity;
    cb->head = i % new_capacity;
    cb->tail = 0;
    cb->count = i;

    // 重新计算统计值
    cb->sum = 0;
    cb->min = INT_MAX;
    cb->max = INT_MIN;
    for (int j = 0; j < cb->count; j++) {
        double val = cb->buffer[j];
        cb->sum += val;
        if (val < cb->min) cb->min = val;
        if (val > cb->max) cb->max = val;
    }

    return true;
}

// 添加数据到缓冲区
void cycle_buffer_push(CycleBuffer *cb, double value) {
    if (!cb || cb->capacity == 0) return;

    if (cb->count == cb->capacity) {
        // 缓冲区已满，覆盖最旧数据（先更新统计值）
        cb->sum -= cb->buffer[cb->head];
        if (cb->buffer[cb->head] == cb->min || cb->buffer[cb->head] == cb->max) {
            // 如果被覆盖的是当前极值，需要重新扫描
            cb->min = INT_MAX;
            cb->max = INT_MIN;
            for (int i = 0; i < cb->count; i++) {
                int idx = (cb->tail + i) % cb->capacity;
                double val = cb->buffer[idx];
                if (val < cb->min) cb->min = val;
                if (val > cb->max) cb->max = val;
            }
        }
    } else {
        cb->count++;
    }

    // 写入新数据并更新统计值
    cb->buffer[cb->head] = value;
    cb->sum += value;
    if (value < cb->min) cb->min = value;
    if (value > cb->max) cb->max = value;

    // 移动头部指针
    cb->head = (cb->head + 1) % cb->capacity;
    if (cb->count == cb->capacity) {
        cb->tail = cb->head;  // 保持 tail 指向最旧数据
    }
}

// 从缓冲区移除最旧数据
double cycle_buffer_pop(CycleBuffer *cb) {
    if (!cb || cb->count == 0) return INT_MIN;

    double value = cb->buffer[cb->tail];
    cb->sum -= value;
    cb->count--;

    // 如果移除的是极值，需要重新扫描
    if (value == cb->min || value == cb->max) {
        cb->min = INT_MAX;
        cb->max = INT_MIN;
        for (int i = 0; i < cb->count; i++) {
            int idx = (cb->tail + i + 1) % cb->capacity;
            double val = cb->buffer[idx];
            if (val < cb->min) cb->min = val;
            if (val > cb->max) cb->max = val;
        }
    }

    cb->tail = (cb->tail + 1) % cb->capacity;
    return value;
}

// 获取缓冲区当前有效数据长度
int cycle_buffer_count(const CycleBuffer *cb) {
    return cb ? cb->count : 0;
}

// 计算平均值
double cycle_buffer_avg(const CycleBuffer *cb) {
    if (!cb || cb->count == 0) return 0.0f;
    return (double)cb->sum / cb->count;
}

// 获取最小值
double cycle_buffer_min(const CycleBuffer *cb) {
    return cb && cb->count > 0 ? cb->min : INT_MIN;
}

// 获取最大值
double cycle_buffer_max(const CycleBuffer *cb) {
    return cb && cb->count > 0 ? cb->max : INT_MIN;
}

// 打印缓冲区内容（调试用）
void cycle_buffer_print(const CycleBuffer *cb) {
    if (!cb) return;
    LOG(LOG_DEBUG, "CycleBuffer [%d/%d]: ", cb->count, cb->capacity);
    for (int i = 0; i < cb->count; i++) {
        int idx = (cb->tail + i) % cb->capacity;
        printf("%f ", cb->buffer[idx]);
    }
    printf("\n");
}