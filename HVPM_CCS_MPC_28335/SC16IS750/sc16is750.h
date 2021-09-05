#ifndef __SC16IS750_H
#define __SC16IS750_H


//void sc16_spi_write_data(uint8_t *data, uint8_t len);
/*void sc16_spi_init(void);
void sc16_spi_uninit(void);*/
void sc16_spi_set(void);
void sc16_spi_start(void);
void sc16_spi_stop(void);
void sc16_spi_read_reg(unsigned int reg, unsigned int *regvalue);

#endif
