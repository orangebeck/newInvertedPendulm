#include "xmt_hal_driver.h"
/*----------------------------------------------------define----------------------------------------------------------------*/
/*xmt struct init*/
struct xmt_datapacket* xmt_init()
{
    struct xmt_datapacket* xmt_datapacket;
    xmt_datapacket = (struct xmt_datapacket*) malloc(sizeof(struct xmt_datapacket));
    xmt_datapacket->xmt_head = 0xaa;
    xmt_datapacket->xmt_instruct2 = 0x00;

    return xmt_datapacket;
}

/*param the return buf to struct xmt_datapacket*/
void xmt_parambuf(unsigned char *p, struct xmt_datapacket* xmt_datapacket)
{
    int ret_len;
    xmt_datapacket->xmt_head = *p++;
    xmt_datapacket->xmt_addr = *p++;
    ret_len = (int)*p;
    LOG(LOG_INFO, "xmt_len = %d\n",ret_len);
    xmt_datapacket->xmt_len = *p++;
    xmt_datapacket->xmt_instruct1 = *p++;
    p++;
    for (int i = 5; i < ret_len-1; i++)
    {
        xmt_datapacket->xmt_data[i-5] = *p++;
    }
    xmt_datapacket->xmt_xor = *p;
    LOG(LOG_INFO, "xmt_xor = %02x\n",*p);
    xmt_datapacket->xmt_datalen = ret_len -6;
}

/*XOR check the results*/
void xmt_xorcheck(struct xmt_datapacket* xmt_datapacket)
{
    unsigned char ret;

    ret =  xmt_datapacket->xmt_head ^ xmt_datapacket->xmt_addr ^  xmt_datapacket->xmt_instruct1 ^ xmt_datapacket->xmt_instruct2;
    xmt_datapacket->xmt_len = (char)(xmt_datapacket->xmt_datalen + 6);
    ret = ret ^xmt_datapacket->xmt_len ;
    for (int i = 0; i < xmt_datapacket->xmt_datalen; i++)
    {
        ret = ret ^ xmt_datapacket->xmt_data[i];
    }
    xmt_datapacket->xmt_xor = ret;
}

/*print struct xmt_datapacket content*/
void xmt_print(struct xmt_datapacket* xmt_datapacket)
{
    LOG(LOG_INFO, "head %02x %02x %02x %02x %02x data",xmt_datapacket->xmt_head,xmt_datapacket->xmt_addr,xmt_datapacket->xmt_len,xmt_datapacket->xmt_instruct1,xmt_datapacket->xmt_instruct2);
    if (xmt_datapacket->xmt_datalen >0)
    {
        for (int i = 0; i < xmt_datapacket->xmt_datalen; i++)
        {
            LOG(LOG_INFO, "%02x ",xmt_datapacket->xmt_data[i]);
        }
    }
    LOG(LOG_INFO, "xor %02x\n",xmt_datapacket->xmt_xor);
    
}

/*put all struct xmt_datapacket content in to a list
    return size of content
    WARING: list size 70*/
unsigned int xmt_datainlist(struct xmt_datapacket* xmt_datapacket, unsigned char *p)
{
    xmt_xorcheck(xmt_datapacket);
    memset(p,0x00,SENDDATASIZE);
    p[0] = xmt_datapacket->xmt_head;
    p[1] = xmt_datapacket->xmt_addr;
    p[2] = xmt_datapacket->xmt_len;
    p[3] = xmt_datapacket->xmt_instruct1;
    p[4] = xmt_datapacket->xmt_instruct2;
    for (int i = 0; i < xmt_datapacket->xmt_datalen; i++)
    {
        p[i+5] = xmt_datapacket->xmt_data[i];
    }
    p[5+xmt_datapacket->xmt_datalen]= xmt_datapacket->xmt_xor;
    return (int)xmt_datapacket->xmt_len;
}

/*----------------------------------------------------usage----------------------------------------------------------------*/

/*convert hex data to voltage*/
double xmt_datatonum(struct xmt_datapacket* xmt_datapacket)
{
    double ret_voltage;
    if (xmt_datapacket->xmt_data[0+1] & 0x80)
    {
        xmt_datapacket->xmt_data[0+1] -= 0x80;
        ret_voltage = (double)(xmt_datapacket->xmt_data[0+1] * 256 + xmt_datapacket->xmt_data[1+1] + (xmt_datapacket->xmt_data[2+1] * 256 + xmt_datapacket->xmt_data[3+1]) * 0.0001);
        ret_voltage *= (-1);
    }
    else
    {
        ret_voltage = (double)(xmt_datapacket->xmt_data[0+1] * 256 + xmt_datapacket->xmt_data[1+1] + (xmt_datapacket->xmt_data[2+1] * 256 + xmt_datapacket->xmt_data[3+1]) * 0.0001);
    }
    return ret_voltage;
}

/*convert input to hex data
    way 0 1 2 3
    multi : 0-1= 0 */
void xmt_numtodata(struct xmt_datapacket* xmt_datapacket, unsigned int way, unsigned int multi, double input,...) 
{ 
    int input_int;
    va_list va_voltage;
    if (multi == 0 || multi ==1)
    {
       multi = 1;
       xmt_datapacket->xmt_instruct1 = 0x01;
    }
    else if (multi > 1)
    {
        va_start(va_voltage,input);
    }

    xmt_datapacket->xmt_data[0] = (char)way;
    xmt_datapacket->xmt_datalen = 1+4*multi;
    for (int i = 0; i < multi; i++)
    {
        if (i != 0)
        {
            input  = va_arg(va_voltage,double);
        }
        
        xmt_datapacket->xmt_data[4*i+1] = 0;
        if (input < 0) // f中的内容为负数
        {
            input *= (-1); // 将F中的内容转换为正数
            xmt_datapacket->xmt_data[4*i+1] = 0x80;
        }
        input_int = (int)input;
        xmt_datapacket->xmt_data[4*i+1] = input_int / 256 | xmt_datapacket->xmt_data[4*i+1];
        xmt_datapacket->xmt_data[4*i+2] = input_int % 256;
        input_int = (int)((input - input_int) * 10000 + 0.1); // 加入舍入因子防止转换过程中出现精度损失
        xmt_datapacket->xmt_data[4*i+3] = input_int / 256;
        xmt_datapacket->xmt_data[4*i+4] = input_int % 256;
    }
    
    if (multi > 1)
    {
        va_end(va_voltage);
    }else if (multi == 1)
    {
        xmt_xorcheck(xmt_datapacket);
        //xmt_print(xmt_datapacket);
    }
}

/*3 way voltage to hex data
    voltage = 0
    displacement = 1
    
    Do not show in .h*/
void xmt_3way_voltodata(double input_1,double input_2, double input_3, struct xmt_datapacket* xmt_datapacket, unsigned int mode)
{
    if (mode == 0)
    {
       xmt_datapacket->xmt_instruct1 = 0x02;
    }else if (mode == 1)
    {
        xmt_datapacket->xmt_instruct1 = 0x03;
    }

    xmt_datapacket->xmt_datalen = 12;
}

/*multiply way zero clearing*/
void xmt_clear(struct xmt_datapacket* xmt_datapacket)
{
    xmt_datapacket->xmt_instruct1 = 0x04;
    xmt_datapacket->xmt_datalen = 0;
    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*read voltage and displacement
    read_type:
        0 : read voltage
        1:  read displacement
    channel*/
void xmt_read(struct xmt_datapacket* xmt_datapacket, int read_type, unsigned int  channel)
{
    if (read_type == 0)
    {
        xmt_datapacket->xmt_instruct1 = 0x05;
    }
    else if (read_type == 1)
    {
        xmt_datapacket->xmt_instruct1 = 0x06;
    }
    xmt_datapacket->xmt_data[0] = (char) channel;
    xmt_datapacket->xmt_datalen= 1;

    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*read responce*/
double xmt_decode_responce(struct xmt_datapacket* xmt_datapacket)
{
    double responce;
    if (xmt_datapacket->xmt_instruct1 == 0x05)  //voltage
    {
        responce = xmt_datatonum(xmt_datapacket);
    }
    else if (xmt_datapacket->xmt_instruct1 == 0x06) //distance
    {
        responce = xmt_datatonum(xmt_datapacket);
    }
    return responce;
}

/*read voltage and displacement realtime
    0 : read voltage
    1:  read displacement*/
void xmt_read_realtime(struct xmt_datapacket* xmt_datapacket, int read_type, int channel, int time)
{
    if (read_type == 0)
    {
        xmt_datapacket->xmt_instruct1 = 0x07;
    }
    else if (read_type == 1)
    {
        xmt_datapacket->xmt_instruct1 = 0x08;
    }

    if (time<0 || time>255)
    {
        LOG(LOG_INFO, "time over range");
        exit(EXIT_FAILURE);
    }else
    {
        xmt_datapacket->xmt_data[0] = (char)channel;
        xmt_datapacket->xmt_data[1] = (char)time;
        xmt_datapacket->xmt_datalen = 2;
        xmt_xorcheck(xmt_datapacket);
        //xmt_print(xmt_datapacket);
    }
}

/*decode realtime responce*/
double xmt_decode_realtime_responce(struct xmt_datapacket* xmt_datapacket)
{
    double responce;
    if (xmt_datapacket->xmt_instruct1 == 0x07)  //voltage
    {
        responce = xmt_datatonum(xmt_datapacket);
    }
    else if (xmt_datapacket->xmt_instruct1 == 0x08) //distance
    {
        responce = xmt_datatonum(xmt_datapacket);
    }
    return responce;
}

/*stop reading*/
void xmt_stop_read(struct xmt_datapacket* xmt_datapacket)
{
    xmt_datapacket->xmt_instruct1 = 0x0b;
    xmt_datapacket->xmt_datalen = 0;
    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*set open-close loop
    O : open loop
    C : close loop*/
void xmt_ocloop(struct xmt_datapacket* xmt_datapacket, char OCmode, unsigned int channel)
{
    xmt_datapacket->xmt_instruct1 = 0x12;
    if (OCmode == 'O')
    {
        xmt_datapacket->xmt_data[0] = (unsigned char)channel;
        xmt_datapacket->xmt_data[1] = 'O';
    }else if (OCmode == 'C')
    {
        xmt_datapacket->xmt_data[0] = (unsigned char)channel;
        xmt_datapacket->xmt_data[1] = 'C';
    }
    xmt_datapacket->xmt_datalen = 2;
    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*read open-close loop*/
void xmt_read_ocloop(struct xmt_datapacket* xmt_datapacket, unsigned int channel)
{
    xmt_datapacket->xmt_instruct1 = 0x13;
    xmt_datapacket->xmt_data[0] = (char)channel;
    xmt_datapacket->xmt_datalen = 1;
    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*decode open-close loop
    ret:
        O : open loop
        C : close loop*/
char xmt_dcode_ocloop(struct xmt_datapacket* xmt_datapacket)
{
    LOG(LOG_INFO, "open close loop= %c\n",xmt_datapacket->xmt_data[1] );
    return xmt_datapacket->xmt_data[1];
}

/*inquire the addr*/
void xmt_addrinquire(struct xmt_datapacket* xmt_datapacket)
{
    xmt_datapacket->xmt_addr = 0x00;
    xmt_datapacket->xmt_instruct1 = 0x2f;
    xmt_datapacket->xmt_datalen = 0;
    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*send single waveform
    channel
    waveform : 'Z':正弦波,'F':方波,'S':三角波,'J':锯齿波
    peak to peak value:
    frequency:
    offset:
    mode:
        0 : voltage
        1 : displacement*/
void xmt_send_waveform(struct xmt_datapacket* xmt_datapacket, int channel, unsigned char waveform, double pp, double freq, double offset, int mode)
{
    xmt_datapacket->xmt_instruct1 = (char) (mode+12);
    xmt_numtodata(xmt_datapacket,channel,3,pp,freq,offset);
    for (int i = xmt_datapacket->xmt_datalen++; i > 0 ; i--)
    {
        xmt_datapacket->xmt_data[i+1] = xmt_datapacket->xmt_data[i];
    }
    xmt_datapacket->xmt_data[1] = waveform;
    xmt_xorcheck(xmt_datapacket);
    xmt_print(xmt_datapacket);
}

/*stop waveform*/
void xmt_stop_waveform(struct xmt_datapacket* xmt_datapacket)
{
    xmt_datapacket->xmt_instruct1 = 0x0e;
    xmt_datapacket->xmt_datalen = 0;

    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*read way displacement
    mode : 0 - lower  1-ceiling*/
void xmt_read_displacement(struct xmt_datapacket* xmt_datapacket, unsigned int mode, unsigned int way)
{
    if (mode == 0)
    {
        xmt_datapacket->xmt_instruct1 = 0x23;
    }else if (mode == 1)
    {
        xmt_datapacket->xmt_instruct1 = 0x1b;
    }
    
    xmt_datapacket->xmt_data[0] = (char)way;
    xmt_datapacket->xmt_datalen = 1;
    xmt_xorcheck(xmt_datapacket);
    //xmt_print(xmt_datapacket);
}

/*return way displacement ceiling*/
double xmt_decode_displacement(struct xmt_datapacket* xmt_datapacket)
{
    double ret;
    ret = xmt_datatonum(xmt_datapacket);
    if (xmt_datapacket->xmt_instruct1 == 0x23)
    {
       xmt_datapacket->lower = ret;
    }else if (xmt_datapacket->xmt_instruct1 == 0x1b)
    {
        xmt_datapacket->upper = ret;
    }
    return ret;
}



