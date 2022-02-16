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

#ifndef RTC_H
#define RTC_H

#include <inttypes.h>

struct rtc_time {
  uint8_t tm_sec;
  uint8_t tm_min;
  uint8_t tm_hour;
  uint8_t tm_mday;
  uint8_t tm_mon;
  uint8_t tm_year;
  uint8_t tm_wday;
};

void rtc_init();

//void handle_rtc_operation(uint8_t opcode, uint8_t *data, uint8_t size);

void rtc_set_date(uint8_t *data);

void rtc_get_date(uint8_t *data);

#endif //RTC_H