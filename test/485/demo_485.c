 #include <stdio.h>
 #include <unistd.h>
 #include <string.h>
 #include <stdlib.h>
 #include <errno.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <termios.h>

 #define SERVER_GPIO_INDEX   "403"
 const char UT_REGISTERS_TAB[] = { 0x02, 0x43, 0xB0, 0x01,0x03, 0xF2};

void config_tty(int fd, struct termios *termios)
{
    // 清空串口接收缓冲区
    tcflush(fd, TCIOFLUSH);
    // 获取串口参数opt
    tcgetattr(fd, termios);

    // 设置串口输出波特率
    cfsetospeed(termios, B115200);
    // 设置串口输入波特率
    cfsetispeed(termios, B115200);
    
    // 设置数据位数
    termios->c_cflag &= ~CSIZE;
    termios->c_cflag |= CS8;
    // 校验位
    termios->c_cflag &= ~PARENB;
    termios->c_iflag &= ~INPCK;
    
    // 禁用软件流控制
    termios->c_iflag &= ~(IXON | IXOFF | IXANY | PARMRK);
    termios->c_iflag &= ~ICRNL; // 禁用回车符转换
    termios->c_iflag &= ~INLCR; // 禁止NL转CR
    termios->c_iflag &= ~IGNCR; // 禁止忽略CR
    
    // 设置停止位
    termios->c_cflag &= ~CSTOPB;

    // 重要：禁用所有信号处理和特殊字符处理
    termios->c_lflag &= ~(ISIG | ECHO | ICANON | IEXTEN);
    
    // 重要：禁用所有控制字符的特殊处理
    termios->c_cc[VMIN] = 1;    // 最小读取字符数
    termios->c_cc[VTIME] = 0;   // 读取超时（无限制）
    
    // 禁用中断字符的特殊处理（0x03）
    termios->c_cc[VINTR] = _POSIX_VDISABLE;
    // 禁用其他可能影响的控制字符
    termios->c_cc[VQUIT] = _POSIX_VDISABLE;
    termios->c_cc[VSUSP] = _POSIX_VDISABLE;
    termios->c_cc[VSTART] = _POSIX_VDISABLE;
    termios->c_cc[VSTOP] = _POSIX_VDISABLE;
    termios->c_cc[VEOF] = _POSIX_VDISABLE;
    termios->c_cc[VEOL] = _POSIX_VDISABLE;

    // 禁用输出处理
    termios->c_oflag &= ~OPOST;
    termios->c_oflag &= ~ONLCR;  // 禁止输出时换行转回车换行

    // 使能接收
    termios->c_cflag |= CREAD;
    
    if (tcsetattr(fd, TCSANOW, termios) < 0)
    {
        printf("XMT fail to config tty\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("XMT device set to 115200bps,8N1 (raw mode)\n");
    }
}
 
 static int _server_ioctl_init(void)
 {
     int fd;
     //index config
     fd = open("/sys/class/gpio/export", O_WRONLY);
     if(fd < 0)
         return 1;

     write(fd, SERVER_GPIO_INDEX, strlen(SERVER_GPIO_INDEX));
     close(fd);

     //direction config
     fd = open("/sys/class/gpio/gpio" SERVER_GPIO_INDEX "/direction", O_WRONLY);
     if(fd < 0)
         return 2;

     write(fd, "out", strlen("out"));
     close(fd);

     return 0;
 }

 static int _send_ioctl(int fd)
 {
     write(fd, "1", 1);
     return 0;
 }

 static int _receive_ioctl(int fd)
 {
     write(fd, "0", 1);
     return 0;
 }

 int main(int argc, char*argv[])
 {
     int rc;
     int i;
     int fd;
     char path[] = "/dev/ttySTM1";
     char buf[1024] = "tty send test.\n";
     char buffer[1024];
     struct termios termios_tty;
     fd = open(path, O_RDWR);
     config_tty(fd, &termios_tty);


     if(fd < 0)
         return 1;
    int res = 0;
 
    usleep(500000);
    for(;;)
    {   
        write(fd, UT_REGISTERS_TAB, 6);
        memset(buffer,0,1024);
        res = read(fd, buffer, 1024);
        if (res > 0) {
            printf("Received %d bytes: ", res);
            for (int i = 0; i < res; i++) {
                printf("%02X ", buffer[i] & 0xFF);
            }
            printf("\n");
            int number = 0;
            number = (buffer[2] <<8 )|buffer[3];
            if(number > 0xEC78)
            {
                number -= 0x10000;
            }
            if(number > 7000 )
            {
                number = 7000;
            }else if (number < -70000)
            {
                number=-7000;
            }
            printf("length = %f\n",(double)number/1000.0);

            
        }



        usleep(500000);
    }
     
     return 0;
 }