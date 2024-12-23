#include "cl_hal_driver.h"

static int mode_old;
static char buf_old[1024];

int CL_enter_measurement_mode(char* buf)
{
    (buf)[0] = 'R';
    (buf)[1] = '0';
    (buf)[2] = 0x0D;
    memcpy(&buf_old, buf,3);
    return 3;
}

int CL_init_data_storage(char* buf)
{
    (buf)[0] = 'D';
    (buf)[1] = 'C';
    (buf)[2] = 0x0D;
    memcpy(&buf_old, buf,3);
    return 3;
}

int CL_start_data_storage(char* buf)
{
    (buf)[0] = 'D';
    (buf)[1] = 'S';
    (buf)[2] = 0x0D;
    memcpy(&buf_old, buf,3);
    return 3;
}

int CL_stop_data_storage(char* buf)
{
    (buf)[0] = 'D';
    (buf)[1] = 'T';
    (buf)[2] = 0x0D;
    memcpy(&buf_old, buf,3);
    return 3;
}

int CL_auto_zero(char* buf, int state, int outway)
{
    (buf)[0] = 'Z';
    (buf)[1] = 'S';
    (buf)[2] = ',';
    (buf)[3] = (char)state;
    (buf)[4] = ',';
    (buf)[5] = (char)outway;
    (buf)[6] = 0x0D;
    memcpy(&buf_old, buf,7);
    return 7;
}

int CL_output_measure_value(char* buf, int mode, int outway)
{
    (buf)[0] = 'M';
    (buf)[1] = 'S';
    (buf)[2] = ',';
    (buf)[3] = (char)mode;
    (buf)[4] = ',';
    (buf)[5] = (char)outway;
    (buf)[6] = 0x0D;
    memcpy(&buf_old, buf,7);
    return 7;
}

int CL_ret_data(char *buf)
{
    int err_num;
    if ((buf)[0] == 'E' && (buf)[1] == 'R')
    {
        err_num = (int)(buf)[6] * 10 + (int)(buf)[7];
        switch (err_num)
        {
        case errnum72:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err72);
            break;
        case errnum73:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err73);
            break;
        case errnum74:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err74);
            break;
        case errnum81:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err81);
            break;
        case errnum82:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err82);
            break;
        case errnum83:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err83);
            break;
        case errnum84:
            printf("error name %c%c %s\n",(buf)[3],(buf)[4],err84);
            break;
        default:
             printf("unknow error %c%c error code %c%c\n",(buf)[3],(buf)[4],(buf)[6],(buf)[7]);
            break;
        }
        return -1;
    }else if (((buf)[0] == buf_old[0])&& ((buf)[1] == buf_old[1]))
    {
        //测试
        //printf("CL order send successfully!\n");
        return 0;
    }
    return -1;
}

float CL_decode_value(char* buf)
{
    float ret;
    ret =((buf)[4]-48)*10+((buf)[5]-48)+((buf)[7]-48)*0.1+((buf)[8]-48)*0.01+((buf)[9]-48)*0.001+((buf)[10]-48)*0.0001;
    if ((buf)[3] == 0x2D)
    {
        ret = -ret;
    }
    return ret;
}