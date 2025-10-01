#include "cycleBuffer.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main() {
    CycleBuffer *cb = cycle_buffer_init(20);
    int i = 0;
    while (1)
    {
        cycle_buffer_push(cb, i);
        cycle_buffer_print(cb);
        printf("i = %d, cycleBuffer sum = %f\n",i,cycle_buffer_avg(cb));
        i++;
        sleep(1);
    }
    
}