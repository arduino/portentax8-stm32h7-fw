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

#include "h7_handler.h"

#include <string.h>

#include "debug.h"
#include "system.h"
#include "opcodes.h"
#include "m4_util.h"
#include "peripherals.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#ifndef REALVERSION
#define REALVERSION "dev " __DATE__ " " __TIME__
#endif

/**************************************************************************************
 * GLOBAL CONST
 **************************************************************************************/

char const __attribute__((section (".fw_version_section"))) REAL_VERSION_FLASH[] = REALVERSION;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int h7_handler(uint8_t opcode, uint8_t *data, uint16_t size)
{
  if (opcode == FW_VERSION)
  {
    const char* version = REAL_VERSION_FLASH;
    return enqueue_packet(PERIPH_H7, FW_VERSION, strlen(version), (void*)version);
  }
  else if (opcode == BOOT_M4)
  {
    int m4_booted_correctly = is_m4_booted_correctly();
    return enqueue_packet(PERIPH_H7, BOOT_M4, sizeof(m4_booted_correctly), &m4_booted_correctly);
  }
  else {
    dbg_printf("h7_handler: error invalid opcode (:%d)\n", opcode);
    return 0;
  }
}
