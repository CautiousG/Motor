//
// Included Files
//

// GPIO32   I2C/SPI 0:SPI 1:I2C
// GPIO33   CS 0:active 1:inactive

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "sc16is750.h"
#include "f28335_spi.h"

#define RHR 0x00
#define THR 0x00

#define IER 0x01

#define FCR 0x02
#define IIR 0x02

#define LCR 0x03
#define MCR 0x04
#define LSR 0x05

#define MSR 0x06
#define SPR 0x07
#define TCR 0x06
#define TLR 0x07

#define TXLVL 0x08
#define RXLVL 0x09

#define IODir 0x0A
#define IOState 0x0B
#define IOIntEna 0x0C
#define IOControl 0x0E

#define EFCR 0x0F

//-------------------------
//Special register set
//-------------------------
#define DLL 0x00
#define DLH 0x01

//-------------------------
//Enhanced register set
//-------------------------
#define EFR 0x02
#define Xon1 0x04
#define Xon2 0x05
#define Xoff1 0x06
#define Xoff2 0x07

void sc16_spi_start()
{
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
}

void sc16_spi_stop()
{
    GpioDataRegs.GPBSET.bit.GPIO33 = 1;
}

Uint16 Tdata = 0x0000;

void sc16_spi_write_reg(Uint16 reg, Uint16 value)
{
    sc16_spi_start();
    reg = (reg << 3);
    reg = (reg & 0x007F);
    //reg = (reg & 0x00FF);
    spi_send_byte(reg);
    spi_send_byte(value);
    sc16_spi_stop();
    DELAY_US(20);
}

void sc16_spi_read_reg(Uint16 reg, Uint16 *regvalue)
{
    sc16_spi_start();

    reg = (reg << 3);
    reg = (reg | 0x0080);

    spi_send_byte(reg);

    //DELAY_US(20);

    *regvalue = spi_receive_byte(0x00);
    sc16_spi_stop();
}

void sc16_spi_set(void)
{
    // 做一个复位
    GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;;
    DELAY_US(10000);
    GpioDataRegs.GPCSET.bit.GPIO87 = 1;
    DELAY_US(10000);

    //sc16_spi_write_reg(0x01, 0x01);
    sc16_spi_write_reg(DLL, 0x01); // 921600
    sc16_spi_write_reg(DLH, 0x00);

    sc16_spi_write_reg(LCR, 0xBF);// access EFR register --
    sc16_spi_write_reg(EFR, 0X10);// enable enhanced registers--

    sc16_spi_write_reg(MCR, 0x04);//--

    sc16_spi_write_reg(TLR, 0X00);//--
    sc16_spi_write_reg(TCR, 0X04);//--

    sc16_spi_write_reg(LCR, 0x03); // 8 bit,no parity,1 stop bits
    sc16_spi_write_reg(FCR, 0x06); // enable ,reset FIFO
    sc16_spi_write_reg(FCR, 0x01); // enable ,FIFOs

    sc16_spi_write_reg(IER, 0x01); // en int
}


//
// End of file
//

