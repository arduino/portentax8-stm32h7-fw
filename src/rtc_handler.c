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

#include "rtc_handler.h"

#include "rtc.h"
#include "debug.h"
#include "opcodes.h"

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int rtc_handler(uint8_t opcode, uint8_t *data, uint16_t size)
{
  if (opcode == SET_DATE)
    return rtc_set_date(data);
  else if (opcode == GET_DATE)
    return rtc_get_date(data);
  else
    dbg_printf("rtc_handler: error invalid opcode (:%d)\n", opcode);

  return 0;
}
