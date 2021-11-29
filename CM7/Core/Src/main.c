#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "peripherals.h"
#include "ringbuffer.h"
#include "can_api.h"
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

//#define PORTENTA_DEBUG_WIRED

#undef DEBUG

volatile bool trigger_irq = false;

/*
volatile bool can1_rx_irq = false;
volatile bool can2_rx_irq = false;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (hfdcan == &hfdcan1) {
      can1_rx_irq = true;
    }
    if (hfdcan == &hfdcan2) {
      can2_rx_irq = true;
    }
  }
}
*/

void peripheral_init() {

  uart_init();

  gpio_init();

  pwm_init();

  dma_init();

  timer_init();

  rtc_init();

  spi_init();

  adc_init();

  canInit();
}

void handle_data() {

  __WFI();

  //serial_rpc_available();
  watchdog_refresh();

  if (uart_data_available()) {
    uart_handle_data();
  }

  if (virtual_uart_data_available()) {
    virtual_uart_handle_data();
  }

  can_handle_data();

  dma_handle_data();

/*
  if (trigger_irq) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
    trigger_irq = false;
  }
*/

}


int main(void) {

  system_init();

  peripheral_init();

  disableCM4Autoboot();

  gpio_set_initial_config();

  printf("Portenta X8 - STM32H7 companion fw - %s %s\n", __DATE__, __TIME__);

  try_execute_m4_app();

  watchdog_init(IWDG_PRESCALER_16);

#ifdef PORTENTA_DEBUG_WIRED
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 1);
#endif

  while (1) {

    handle_data();

  }
}
