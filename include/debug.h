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

#ifndef PORTENTAX8_STM32H7_FW_DEBUG_H
#define PORTENTAX8_STM32H7_FW_DEBUG_H

/**************************************************************************************
 * DEFINE
 **************************************************************************************/
#include <stdio.h>

#ifdef DEBUG
#define dbg_printf(...)  printf(__VA_ARGS__)
#else
#define dbg_printf(...)  do {} while (0)
#endif

#endif /* PORTENTAX8_STM32H7_FW_DEBUG_H */
