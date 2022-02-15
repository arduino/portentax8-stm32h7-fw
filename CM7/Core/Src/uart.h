/*
 * Firmware for the Portenta X8 STM32H747AIIX/Cortex-M7 core.
 * Copyright (C) 2022 Arduino (http://www.arduino.cc/)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <inttypes.h>

#ifndef UART_H
#define UART_H

enum UARTParity {
  PARITY_EVEN = 0,
  PARITY_ODD,
  PARITY_NONE,
};

void uart_init();

//void uart_handler(uint8_t opcode, uint8_t *data, uint8_t size);

void uart_configure(uint8_t *data);

int uart_write(uint8_t *data, uint16_t size);

int uart_write_with_timeout(uint8_t *data, uint16_t size, uint32_t timeout);

int uart_data_available();

void uart_handle_data();

int virtual_uart_data_available();

void virtual_uart_handle_data();

void UART2_enable_rx_irq();

#endif//UART_H