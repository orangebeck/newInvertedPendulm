#ifndef __XMT_hal_driver_H__
#define __XMT_hal_driver_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define SENDDATASIZE 1024

struct xmt_datapacket
{
    unsigned char xmt_head; //start byte: 0xaa                0
    unsigned char xmt_addr; //addr                                      1
    unsigned char xmt_len;  //the packet length             2
    unsigned char xmt_instruct1;//                                         3
    unsigned char xmt_instruct2;//                                         4
    unsigned char xmt_data[64]; //data segment
    int xmt_datalen;
    unsigned char xmt_xor; 
    double upper;
    double lower;
};

/*xmt struct init*/
struct xmt_datapacket* xmt_init();

/*param the return buf to struct xmt_datapacket*/
void xmt_parambuf(unsigned char *p, struct xmt_datapacket* xmt_datapacket);

/*convert hex data to voltage*/
double xmt_datatonum(struct xmt_datapacket* xmt_datapacket);

/*convert input to hex data
    way 0 1 2 3 */
void xmt_numtodata(struct xmt_datapacket* xmt_datapacket, unsigned int way, unsigned int multi, double input,...);

/*XOR check the results*/
void xmt_xorcheck(struct xmt_datapacket* xmt_datapacket);

/*multiply way zero clearing*/
void xmt_clear(struct xmt_datapacket* xmt_datapacket);

/*read voltage and displacement
    read_type:
        0 : read voltage
        1:  read displacement
    channel*/
void xmt_read(struct xmt_datapacket* xmt_datapacket, int read_type,  unsigned int  channel);

/*read responce*/
double xmt_decode_responce(struct xmt_datapacket* xmt_datapacket);

/*read voltage and displacement realtime
    0 : read voltage
    1:  read displacement*/
void xmt_read_realtime(struct xmt_datapacket* xmt_datapacket, int read_type, int channel, int time);

/*decode realtime responce*/
double xmt_decode_realtime_responce(struct xmt_datapacket* xmt_datapacket);

/*stop reading*/
void xmt_stop_read(struct xmt_datapacket* xmt_datapacket);

/*set open-close loop
    O : open loop
    C : close loop*/
void xmt_ocloop(struct xmt_datapacket* xmt_datapacket, char OCmode, unsigned int channel);

/*read open-close loop*/
void xmt_read_ocloop(struct xmt_datapacket* xmt_datapacket, unsigned int channel);

/*decode open-close loop
    ret:
        O : open loop
        C : close loop*/
char xmt_dcode_ocloop(struct xmt_datapacket* xmt_datapacket);

/*send single waveform
    channel
    waveform : 'Z':正弦波,'F':方波,'S':三角波,'J':锯齿波
    peak to peak value:
    frequency:
    offset:
    mode:
        0 : voltage
        1 : displacement*/
void xmt_send_waveform(struct xmt_datapacket* xmt_datapacket, int channel, unsigned char waveform, double pp, double freq, double offset, int mode);

/*stop waveform*/
void xmt_stop_waveform(struct xmt_datapacket* xmt_datapacket);

/*read way displacement
    mode : 0 - lower  1-ceiling*/
void xmt_read_displacement(struct xmt_datapacket* xmt_datapacket, unsigned int mode, unsigned int way);

/*return way displacement ceiling*/
double xmt_decode_displacement(struct xmt_datapacket* xmt_datapacket);

/*print struct xmt_datapacket content*/
void xmt_print(struct xmt_datapacket* xmt_datapacket);

/*put all struct xmt_datapacket content in to a list
    return size of content
    WARING: list size 70*/
unsigned int xmt_datainlist(struct xmt_datapacket* xmt_datapacket, unsigned char *p);

/*inquire the addr*/
void xmt_addrinquire(struct xmt_datapacket* xmt_datapacket);

#endif