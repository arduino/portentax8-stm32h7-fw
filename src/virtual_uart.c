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
#include "double-buffer.h"

#include "debug.h"
#include <stdint.h>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

//ring_buffer_t virtual_uart_ring_buffer; /* extern'ally referenced in rpc.c */

static uint8_t tx_virtual_uart_1[DBL_BUFF_UART_SIZE];
static uint8_t tx_virtual_uart_2[DBL_BUFF_UART_SIZE];

DblBuffer_t dblBuffer_VIRT_UART;


/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void virtual_uart_init()
{
  dblBuffer_init(&dblBuffer_VIRT_UART, NULL, NULL, tx_virtual_uart_1, tx_virtual_uart_2, 0, DBL_BUFF_UART_SIZE);
  //ring_buffer_init(&virtual_uart_ring_buffer);
}

int virtual_uart_data_available() {
  if (dblBuffer_numTxToRemove(&dblBuffer_VIRT_UART) > 0 ||
      dblBuffer_getTXtoWriteWhenWriting(&dblBuffer_VIRT_UART) > 0) {
        return 1;
  }
  return 0;
}

int enqueue_remaining() {
  int num_to_tx = dblBuffer_numTxToRemove(&dblBuffer_VIRT_UART);
  if(num_to_tx > 0) {
    uint16_t bytes_to_send = min((SPI_DMA_BUFFER_SIZE/2),num_to_tx);
    enqueue_packet(PERIPH_VIRTUAL_UART, DATA, bytes_to_send, dblBuffer_getTXtoSend(&dblBuffer_VIRT_UART, 1));
    dblBuffer_increaseTXtoWritePosWhenRemoving(&dblBuffer_VIRT_UART, bytes_to_send);
  }
  return num_to_tx;
}

int virtual_uart_handle_data() {
  if(enqueue_remaining() <= 0) {
    __disable_irq();
    dblBuffer_swapTX(&dblBuffer_VIRT_UART);
    __enable_irq();
    enqueue_remaining();
  }
  return 0;
}
