//###########################################################################
//
// FILE:   DSP2833x_Spi.c
//
// TITLE:  DSP2833x SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2833x/F2823x Header Files and Peripheral Examples V142 $
// $Release Date: November  1, 2016 $
// $Copyright: Copyright (C) 2007-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "f28335_SCI.h"

#define CPU_FREQ     150E6
#define LSPCLK_FREQ  CPU_FREQ/4
#define SCI_FREQ     921600
#define SCI_PRD      (LSPCLK_FREQ/(SCI_FREQ*8))-1

void scib_fifo_init()
{
   ScibRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
   ScibRegs.SCICTL1.all =0x0003;   // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
   //ScibRegs.SCICTL2.bit.TXINTENA =1;
   //ScibRegs.SCICTL2.bit.RXBKINTENA =1;
   ScibRegs.SCIHBAUD    =0x0000;
   ScibRegs.SCILBAUD    =0x0001;//SCI_PRD   // 0x0003:1171875   0x0002:1562500  0x0001:2343750  0x0000:4687500
   //ScibRegs.SCICCR.bit.LOOPBKENA =1; // Enable loop back
   //ScibRegs.SCIFFTX.all=0xC028;
   //ScibRegs.SCIFFRX.all=0x0028;
   ScibRegs.SCIFFTX.all =0xE040;
   ScibRegs.SCIFFRX.all =0x204f;
   ScibRegs.SCIFFCT.all =0x00;

   ScibRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
   //ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
   //ScibRegs.SCIFFRX.bit.RXFIFORESET=1;
}

void scib_xmit(int a)
{
    ScibRegs.SCITXBUF = a;
}

Uint16 *temp;
Uint16 sdata;
Uint16 n = 0;
void scib_xmit_float(float *pdata,int num)
{
    n = 0;
    num = 2*num;
    temp = (Uint16*)pdata;
    while(n<num)
    {
        sdata = *(temp+n);
        ScibRegs.SCITXBUF = sdata&0x00FF;
        ScibRegs.SCITXBUF = (sdata&0xFF00)>>8;
        n++;
    }
    /*ScibRegs.SCITXBUF = 0x1357;
    ScibRegs.SCITXBUF = 0x3333;
    ScibRegs.SCITXBUF = 0x7777;
    ScibRegs.SCITXBUF = 0xFFFF;*/
}

//
// End of file
//

