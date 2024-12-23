#ifndef __CL_HAL_DRIVER_H__
#define __CL_HAL_DRIVER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

enum command_b{
    OFF = 48,
    ON
};

enum command_x{
    out1=49,
    out2,
    out3,
    out4,
    out5,
    out6,
    out7,
    out8
};

enum command_m{
    value = 48,
    value_result,
    value_judge,
    value_result_judge,
    count_value,
    count_value_result,
    count_value_judge,
    count_value_result_judge
};

enum return_error{
    errnum72 = 72,
    errnum73,
    errnum74,
    errnum81=81,
    errnum82,
    errnum83,
    errnum84
};

static char  err72[] = "error code 72: Timeout error";
static char  err73[] = "error code 73: Command length error";
static char  err74[] = "error code 74: Undefined command error";
static char  err81[] = "error code 81: Status error";
static char  err82[] = "error code 82: Wrong number of parameters";
static char  err83[] = "error code 83: Parameter range error";
static char  err84[] = "error code 84: Command inherent error (please refer to the details of each command)";

int CL_enter_measurement_mode(char *buf);

int CL_init_data_storage(char *buf);

int CL_start_data_storage(char *buf);

int CL_stop_data_storage(char *buf);

int CL_auto_zero(char *buf, int state, int outway);

int CL_output_measure_value(char *buf, int mode, int outway);

int CL_ret_data(char *buf);

float CL_decode_value(char *buf);
#endif