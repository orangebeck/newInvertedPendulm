#ifndef __MOTOR_H__
#define __MOTOR_H__

#define MOTOR_DEVICE_NAME "/dev/motor_misc"   //设备名称
#define MOTOR_IOCTL_CMD_SET_VALUE _IOW('M', 1, int)   //ioctrl魔数

typedef struct motor
{
    float led;
    struct motor_ctrl 
    {
        int dir;
        int enx;
        int micro;
        int pul;
    }motor_ctrl;
}motor;

#endif