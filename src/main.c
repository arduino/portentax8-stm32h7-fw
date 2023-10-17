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

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "peripherals.h"
#include "ringbuffer.h"
#include "can.h"
#include "rpc.h"
#include "adc.h"
#include "uart.h"
#include "pwm.h"
#include "gpio.h"
#include "timer.h"
#include "rtc.h"
#include "spi.h"
#include "system.h"
#include "watchdog.h"
#include "m4_utilities.h"

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void peripheral_init() {

  uart_init();

  gpio_init();

  pwm_init();

  dma_init();

  timer_init();

  rtc_init();

  spi_init();

  adc_init();

  can_init();
}

void handle_data()
{
  __WFI();

  watchdog_refresh();

  int bytes_enqueued = 0;

  if (uart_data_available())
    bytes_enqueued += uart_handle_data();

  if (virtual_uart_data_available())
    bytes_enqueued += virtual_uart_handle_data();

  bytes_enqueued += can_handle_data();
  bytes_enqueued += gpio_handle_data();
  bytes_enqueued += dma_handle_data();

  if (bytes_enqueued)
    trigger_packet();
}

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(void) {

  system_init();

  peripheral_init();

  disableCM4Autoboot();

  gpio_set_initial_config();

  extern char const REAL_VERSION_FLASH[];
  printf("Portenta X8 - STM32H7 companion fw - %s\n", REAL_VERSION_FLASH);

  try_execute_m4_app();

  watchdog_init(IWDG_PRESCALER_16);

  while (1) {

    handle_data();

  }
}
