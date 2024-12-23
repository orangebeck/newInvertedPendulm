#include "queue.h"

void initializeQueue(CircularQueue *q) {
    q->front = 0;
    q->rear = -1;
    q->count = 0;
}

bool isFull(CircularQueue *q) {
    return q->count == QUEUE_SIZE;
}

bool isEmpty(CircularQueue *q) {
    return q->count == 0;
}

bool enqueue(CircularQueue *q, double item) {
    if (isFull(q)) {
        return false;
    }
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->items[q->rear] = item;
    q->count++;
    return true;
}

bool dequeue(CircularQueue *q, double *item) {
    if (isEmpty(q)) {
        return false;
    }
    *item = q->items[q->front];
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->count--;
    return true;
}
