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

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "virtual_uart.h"

#include "stm32h7xx_hal.h"

#include "system.h"
#include "opcodes.h"
#include "ringbuffer.h"
#include "peripherals.h"

#include "debug.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ring_buffer_t virtual_uart_ring_buffer; /* extern'ally referenced in rpc.c */

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void virtual_uart_init()
{
  dbg_printf("r r r r ring buffer init\n");
  ring_buffer_init(&virtual_uart_ring_buffer);
}

int virtual_uart_data_available()
{
  return !ring_buffer_is_empty(&virtual_uart_ring_buffer);
}

int virtual_uart_handle_data()
{
  uint8_t temp_buf[RING_BUFFER_SIZE];
  ring_buffer_size_t send_num = min((SPI_DMA_BUFFER_SIZE/2), ring_buffer_num_items(&virtual_uart_ring_buffer));
  __disable_irq();
  int const cnt = ring_buffer_dequeue_arr(&virtual_uart_ring_buffer, (char *)temp_buf, send_num);
  __enable_irq();
  
  dbg_printf("M4 to H7: ");
  
  for(int i = 0; i < cnt; i++) {
    if(*(temp_buf + i) < 0) {
      dbg_printf("0");
    }
    dbg_printf("%X ", *(temp_buf+i));
  }
  dbg_printf("\n");
  
  
  return enqueue_packet(PERIPH_VIRTUAL_UART, DATA, cnt, temp_buf);
}
