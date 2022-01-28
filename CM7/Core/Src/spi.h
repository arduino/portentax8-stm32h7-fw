#include <inttypes.h>

#ifndef SPI_H
#define SPI_H

void spi_init();

void spi_end();

void spi_transmit_receive(uint8_t peripheral, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t size);

#endif  //SPI_H
