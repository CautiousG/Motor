#ifndef __F28335_SPI_H
#define __F28335_SPI_H

void spi_xmit_16bit(float *pdata,int num);
void spi_xmit_8bit(float *pdata,int num);
void spi_fifo_init(void);
void spi_init_16bit(void);
void spi_init_8bit(void);
void spi_send_byte(unsigned int data);
unsigned int spi_receive_byte(unsigned int Tdata);

#endif
