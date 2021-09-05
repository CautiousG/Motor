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
#include "f28335_SPI.h"

void spi_init_16bit()
{
    SpiaRegs.SPICCR.all = 0x000F;               // Reset on, rising edge, 16-bit char bits
    SpiaRegs.SPICTL.all = 0x0006;               // Enable master mode, normal phase,

    // enable talk, and SPI int disabled.
    SpiaRegs.SPIBRR = 0x0018;                   // LSPCLK = 37.5M Baud = LSPCLK/(SIBRR+1) 7.5Mhz
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    SpiaRegs.SPIPRI.bit.FREE = 1;               // Set so breakpoints don't disturb xmission
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
}

void spi_init_8bit()
{
    SpiaRegs.SPICCR.all = 0x0007;               // Reset on, rising edge, 16-bit char bits
    SpiaRegs.SPICTL.all = 0x0006;               // Enable master mode, normal phase,

    // enable talk, and SPI int disabled.
    SpiaRegs.SPIBRR = 0x0018;                   // LSPCLK = 37.5M Baud = LSPCLK/(SIBRR+1) 7.5Mhz
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    SpiaRegs.SPIPRI.bit.FREE = 1;               // Set so breakpoints don't disturb xmission
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
}


Uint16 *temp;
Uint16 sdata;
Uint16 n = 0;
void spi_xmit_16bit(float *pdata,int num)
{
    n = 0;
    num = 2*num;
    temp = (Uint16*)pdata;
    while(n<num)
    {
        sdata = *(temp+n);
        SpiaRegs.SPITXBUF = sdata;
        n++;
    }
    /*SpiaRegs.SPITXBUF = 0x1111;
    SpiaRegs.SPITXBUF = 0x3333;
    SpiaRegs.SPITXBUF = 0x7777;
    SpiaRegs.SPITXBUF = 0xFFFF;*/
}


void spi_xmit_8bit(float *pdata,int num)
{
    n = 0;
    num = 2*num;
    temp = (Uint16*)pdata;
    while(n<num)
    {
        sdata = *(temp+n);
        SpiaRegs.SPITXBUF = sdata&0xFF00;
        SpiaRegs.SPITXBUF = (sdata&0x00FF)<<8;
        n++;
    }
    /*SpiaRegs.SPITXBUF = 0x1357;
    SpiaRegs.SPITXBUF = 0x3333;
    SpiaRegs.SPITXBUF = 0x7777;
    SpiaRegs.SPITXBUF = 0xFFFF;*/
}

void spi_send_byte(Uint16 data)
{
    SpiaRegs.SPITXBUF = (data&0x00FF)<<8;
}

Uint16 retry = 0;

Uint16 spi_receive_byte(Uint16 Tdata)
{
    retry = 0;
    SpiaRegs.SPITXBUF = (Tdata&0x00FF)<<8;
    /*while(SpiaRegs.SPIFFRX.bit.RXFFST !=1)
    {
        retry++;
        if(retry>2000)
            return 0;
    }*/
    return ((SpiaRegs.SPIRXBUF)&0x00FF);
    //return SpiaRegs.SPIRXBUF;
}

void spi_fifo_init()
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x204f;
    SpiaRegs.SPIFFCT.all=0x0;
}

//
// End of file
//

