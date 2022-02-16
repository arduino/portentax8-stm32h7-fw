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

#ifndef PWM_H
#define PWM_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <inttypes.h>
#include <stdbool.h>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

struct __attribute__((packed, aligned(4))) pwmPacket {
	uint8_t enable: 1;
	uint8_t polarity: 1;
	uint32_t duty: 30;
	uint32_t period: 32;
};

struct __attribute__((packed, aligned(4))) pwmCapture {
	uint8_t enable: 1;
	uint8_t polarity: 1;
	uint32_t duty: 30;
	uint32_t period: 32;
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void pwm_init();

void capturePwm(uint8_t channel);

void configurePwm(uint8_t channel, bool enable, bool polarity, uint32_t duty_ns, uint32_t period_ns);

#endif  //PWM_H