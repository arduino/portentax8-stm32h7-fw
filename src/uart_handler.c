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

#include "uart_handler.h"

#include "uart.h"
#include "debug.h"
#include "opcodes.h"

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int uart_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CONFIGURE)
  {
    uart_configure(data);
  }
  else if (opcode == DATA)
  {
    uart_write(data, size);
  }
  else
  {
    dbg_printf("uart_handler: error invalid opcode (:%d)\n", opcode);
  }
  return 0;
}
