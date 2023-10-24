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

#include "pwm_handler.h"

#include "pwm.h"
#include "debug.h"
#include "opcodes.h"

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int pwm_handler(uint8_t opcode, uint8_t *data, uint16_t size)
{
  if (opcode == CAPTURE)
  {
    uint8_t const channel = opcode & 0x0F;
    if (isValidPwmChannelNumber(channel))
      capturePwm(channel);
    else
      dbg_printf("pwm_handler: invalid PWM channel number provided for mode CAPTURE: %d\n", channel);
  }
  else if (opcode == CONFIGURE)
  {
    uint8_t const channel = opcode;
    struct pwmPacket config = *((struct pwmPacket*)data);
    if (isValidPwmChannelNumber(channel))
      configurePwm(channel, config.enable, config.polarity, config.duty, config.period);
    else
      dbg_printf("pwm_handler: invalid PWM channel number provided for mode PWM: %d\n", channel);
  }
  else
  {
    dbg_printf("pwm_handler: error invalid opcode (:%d)\n", opcode);
  }
  return 0;
}
