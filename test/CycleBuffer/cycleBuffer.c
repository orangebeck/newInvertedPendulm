#include "cycleBuffer.h"

cycleBuffer* initCycleBuffer(int size)
{
    cycleBuffer* cb = (cycleBuffer*)malloc(sizeof(cycleBuffer) + size * sizeof(double));
    memset(cb, 0, sizeof(cycleBuffer) + size * sizeof(double));
    cb->size = size;
    return cb;
}

double putCycleBuffer(cycleBuffer *cb, double data)
{
    int fornt = cb->head;
    int end = cb->tail;

    if(cb->count == cb->size)
    {
        cb->sum -= cb->buffer[end];
        
    }else
    {
        cb->count++;
    }
    cb->buffer[end] = data;
    cb->sum += data;
    cb->tail = (end + 1) % cb->size;
    return cb->sum;
}

void clearCycleBuffer(cycleBuffer *cb)
{
    int size = cb->size;
    memset(cb, 0, sizeof(cycleBuffer) + cb->size * sizeof(double));
    cb->size = size;
}

void destoryCycleBuffer(cycleBuffer *cb)
{
    free(cb);
}