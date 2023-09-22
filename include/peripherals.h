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
  PERIPH_M4 = 0x08,
  PERIPH_H7 = 0x09,
  PERIPH_VIRTUAL_UART = 0x0A,
  PERIPH_SPI2 = 0x0B,
  PERIPH_SPI3 = 0x0C,
};


enum Opcodes {
  CONFIGURE = 0x10,
  DATA = 0x01,
};

enum Opcodes_H7 {
  FW_VERSION = 0x10,
  BOOT_M4 = 0x77,
};

enum Opcodes_UART {
  GET_LINESTATE = 0x20,
};

enum Opcodes_RTC {
  SET_DATE = 0x01,
  GET_DATE = 0x02,
  SET_ALARM = 0x11,
  GET_ALARM = 0x12,
};

enum Opcodes_CAN {
  CAN_TX_FRAME = 0x01,
  CAN_STATUS = 0x40,
  CAN_FILTER = 0x50,
};

enum Opcodes_GPIO {
  DIRECTION = 0x10,
  IRQ_TYPE = 0x11,
  WRITE = 0x20,
  READ = 0x30,
  IRQ_ENABLE = 0x40,
  IRQ_SIGNAL = 0x50,
  IRQ_ACK = 0x60,
};

enum Opcodes_PWM {
  CAPTURE = 0x60,
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

const char* to_peripheral_string(enum Peripherals peripheral);

#endif //PERIPHERALS_H