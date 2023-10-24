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

#ifndef PORTENTAX8_STM32H7_FW_OPCODES_H
#define PORTENTAX8_STM32H7_FW_OPCODES_H

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum Opcodes
{
  CONFIGURE = 0x10,
  DATA      = 0x01,
};

enum Opcodes_H7
{
  FW_VERSION = 0x10,
  BOOT_M4    = 0x77,
};

enum Opcodes_UART
{
  GET_LINESTATE = 0x20,
};

enum Opcodes_RTC
{
  SET_DATE  = 0x01,
  GET_DATE  = 0x02,
  SET_ALARM = 0x11,
  GET_ALARM = 0x12,
};

enum Opcodes_CAN
{
  CAN_TX_FRAME = 0x01,
  CAN_RX_FRAME = 0x01,
  CAN_STATUS   = 0x40,
  CAN_FILTER   = 0x50,
};

enum Opcodes_GPIO
{
  DIRECTION  = 0x10,
  IRQ_TYPE   = 0x11,
  WRITE      = 0x20,
  READ       = 0x30,
  IRQ_ENABLE = 0x40,
  IRQ_SIGNAL = 0x50,
  IRQ_ACK    = 0x60,
};

enum Opcodes_PWM
{
  CAPTURE = 0x60,
};

#endif //PORTENTAX8_STM32H7_FW_OPCODES_H
