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

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <inttypes.h>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum Peripherals {
  PERIPH_ADC = 0x01,
  PERIPH_PWM = 0x02,
  PERIPH_FDCAN1 = 0x03,
  PERIPH_FDCAN2 = 0x04,
  PERIPH_UART = 0x05,
  PERIPH_RTC = 0x06,
  PERIPH_GPIO = 0x07,
  PERIPH_H7 = 0x09,
  PERIPH_VIRTUAL_UART = 0x0A,
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

const char* to_peripheral_string(enum Peripherals peripheral);

#endif //PERIPHERALS_H