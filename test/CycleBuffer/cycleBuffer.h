#ifndef CYCLEBUFFER_H
#define CYCLEBUFFER_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    int size;   // buffer size
    int head;   // index of the first element in the buffer
    int tail;   // index of the next available slot in the buffer
    int count;  // number of elements in the buffer
    double sum;  // sum of all elements in the buffer
    double buffer[];    // the buffer itself
} cycleBuffer;

cycleBuffer* initCycleBuffer(int size);

double putCycleBuffer(cycleBuffer *cb, double data);    // add an element to the buffer

void clearCycleBuffer(cycleBuffer *cb);   // clear the buffer

void destoryCycleBuffer(cycleBuffer *cb);   // delete the buffer

#endif