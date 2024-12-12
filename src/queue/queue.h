#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define QUEUE_SIZE 200

typedef struct {
    double items[QUEUE_SIZE];
    int front;
    int rear;
    int count;
} CircularQueue;

void initializeQueue(CircularQueue *q) ;

bool isFull(CircularQueue *q);

bool isEmpty(CircularQueue *q);

bool enqueue(CircularQueue *q, double item);

bool dequeue(CircularQueue *q, double *item);

#endif