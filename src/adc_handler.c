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

#include "adc_handler.h"

#include "adc.h"
#include "debug.h"
#include "peripherals.h"

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int adc_handler(uint8_t opcode, uint8_t *data, uint16_t size)
{
  if (opcode == CONFIGURE)
  {
    /* Note: ADC currently only supports polling mode.
     * uint16_t adc_sample_rate = *((uint16_t*)data);
     * dbg_printf("Setting ADC samplerate to %d milliseconds\n", adc_sample_rate);
     */
    return 0;
  }
  else if ((opcode >= A0) && (opcode <= A7))
  {
    return get_adc_value(opcode);
  }
  else
  {
    dbg_printf("adc_handler: invalid ADC opcode %02x\n", opcode);
    return 0;
  }
}
