#include "cycleBuffer.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main() {
    cycleBuffer *cb = initCycleBuffer(8);
    int i = 0;
    while (1)
    {
        printf("i = %d, cycleBuffer sum = %f\n",i,putCycleBuffer(cb, 1)/cb->count);
        i++;
        sleep(1);
    }
    
}