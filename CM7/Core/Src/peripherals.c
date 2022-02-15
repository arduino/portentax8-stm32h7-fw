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

#include "peripherals.h"

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