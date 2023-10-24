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

#include "peripherals.h"

#include "debug.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define NUM_PERIPHERAL_CALLBACKS (20)

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

PeriphCallbackFunc PeriphCallbacks[NUM_PERIPHERAL_CALLBACKS];

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

const char* to_peripheral_string(enum Peripherals peripheral) {
  switch (peripheral) {
    case PERIPH_ADC:
      return "ADC";
    case PERIPH_PWM:
      return "PWM";
    case PERIPH_FDCAN1:
      return "FDCAN1";
    case PERIPH_FDCAN2:
      return "FDCAN2";
    case PERIPH_UART:
      return "UART";
    case PERIPH_RTC:
      return "RTC";
    case PERIPH_GPIO:
      return "GPIO";
    case PERIPH_VIRTUAL_UART:
      return "VIRTUAL_UART";
    case PERIPH_H7:
      return "FW";
    default:
      return "UNKNOWN";
  }
}

void peripheral_register_callback(uint8_t const peripheral, PeriphCallbackFunc const func)
{
  PeriphCallbacks[peripheral] = func;
}

int peripheral_invoke_callback(uint8_t const peripheral, uint8_t const opcode, uint8_t * data, uint16_t const size)
{
  if (peripheral >= NUM_PERIPHERAL_CALLBACKS) {
    dbg_printf("error, invalid peripheral id received: %d\n", peripheral);
    return -1;
  }

  /* Obtain the registered callback for the selected peripheral. */
  PeriphCallbackFunc const peripheral_callback = PeriphCallbacks[peripheral];

  /* Invoke the registered callback for the selected peripheral. */
  int const rc = peripheral_callback(opcode, data, size);
  return rc;
}
