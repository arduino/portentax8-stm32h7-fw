#include <inttypes.h>

#ifndef UART_H
#define UART_H

enum UARTParity {
  PARITY_EVEN = 0,
  PARITY_ODD,
  PARITY_NONE,
};

void uart_init();

void uart_configure(uint8_t *data);

int uart_write(uint8_t *data, uint16_t size, uint32_t timeout);

int uart_data_available();

void uart_handle_data();

int virtual_uart_data_available();

void virtual_uart_handle_data();

void UART2_enable_rx_irq();

#endif//UART_H