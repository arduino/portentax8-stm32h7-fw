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

#include "virtual_uart_handler.h"
#include "debug.h"
#include "rpc.h"

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

extern int debug_callback_invocation;
unsigned int debug_size = 0;

int virtual_uart_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  debug_callback_invocation++;
  debug_size += size;
  dbg_printf("X8 to M4: [%i] ", size);
  for(int i = 0; i < size; i++) {
    if(*(data + i) < 0) {
      dbg_printf("0");
    }
    if(i >= 4) {

       dbg_printf("%c ", (char)*(data+i));
    } else  {
      dbg_printf("%X ", *(data+i));
    }
  }
  dbg_printf("\n");


  serial_rpc_write(data, size);
  return 0;
}
