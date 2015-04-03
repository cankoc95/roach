/* 
 * File:   tactile_driver.h
 * Author: jgoldberg
 *
 * Created on April 14, 2014, 12:15 PM
 */

#include "uart.h"
#include <stdio.h>
#include "settings.h"
#include <stdint.h>

#ifndef UART_H
#define	UART_H

//#ifndef TACTILE_DRIVER_H
//#define	TACTILE_DRIVER_H

#define TACTILE_TX_IDLE          0xFF
#define TACTILE_RX_IDLE          0xFF
#define TACTILE_MODE_A           0x41
#define TACTILE_MODE_B           0x42
#define TACTILE_MODE_C           0x43
#define TACTILE_MODE_D           0x44
#define TACTILE_MODE_E           0x45
#define TACTILE_MODE_F           0x46
#define TACTILE_MODE_G           0x47
#define TACTILE_MODE_L           0x4C
#define TACTILE_MODE_S           0x53
#define TACTILE_MODE_T           0x54
#define CTS                      0x5A //'Z'


// buffer lengths
#define SMALL_BUFFER            3
#define LARGE_BUFFER            110

typedef struct {
    uint16_t frame[ROWS*COLS];
} tactileFrame_t;

typedef struct {
    float Fx;
    float Fy;
    float Fz;
    float Mx;
    float My;
    float Mz;
    float *F[6];
} tactileForces;

void tactileInit();
void checkFrameSize();
void handleSkinRequest(unsigned char length,unsigned char *frame,unsigned int src_addr);
unsigned char sendTactileCommand(unsigned char length,unsigned char *frame);
void handleSkinData(unsigned int length, unsigned char *data);
void setRXFlag();
void clearRXFlag();
unsigned char checkRXFlag();
void checkTactileBuffer();
void sendCTS();
int tactileReturnFrame(tactileFrame_t* dst);
void calcForces(tactileFrame_t* frame, tactileForces* F);
void tactilePID(tactileForces* F);
int setLegFreqs(int numfreqs, int* freq);

//#ifdef	__cplusplus
//extern "C" {
//#endif

typedef enum {
    ABC = 0,
    ABCD
} testenum;


//#ifdef	__cplusplus
//}
//#endif

#endif	/* TACTILE_DRIVER_H */

