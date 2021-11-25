#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "peripherals.h"
#include "ringbuffer.h"
#include "can_api.h"
#include "rpc.h"
#include "stm32h7xx_ll_rcc.h"
#include "adc.h"
#include "uart.h"
#include "pwm.h"
#include "gpio.h"
#include "timer.h"
#include "rtc.h"
#include "spi.h"
#include "system.h"
#include "watchdog.h"

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

static FLASH_OBProgramInitTypeDef OBInit;

void disableCM4Autoboot() {
  OBInit.Banks     = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  dbg_printf("OBInit.USERConfig: %X\n", OBInit.USERConfig);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    dbg_printf("Changing option bytes\n");
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    if (HAL_FLASH_OB_Unlock() == HAL_OK)
      if (HAL_FLASH_Unlock() == HAL_OK)
        if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK)
          if (HAL_FLASH_OB_Launch() == HAL_OK)
            if (HAL_FLASH_OB_Lock() == HAL_OK)
              if (HAL_FLASH_Lock() == HAL_OK)
              {
                dbg_printf("Option bytes changed\n");
                dbg_printf("Requires rebooting\n");
                NVIC_SystemReset();
                return;
              }
    dbg_printf("Failed changing option bytes");
  }
}

void peripheral_init() {

  uart_init();

  gpio_init();

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

  // Start DMA on SPI
  //HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)TX_Buffer,
  //                            (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);

  printf("Portenta X8 - STM32H7 companion fw - %s %s\n", __DATE__, __TIME__);

  int m4_app_valid = (((*(__IO uint32_t *) FLASH_BANK2_BASE) & 0xFF000000) == 0x10000000);

  if (m4_app_valid) {
    printf("Boot CM4\n");
    LL_RCC_ForceCM4Boot();
    int ret = serial_rpc_begin();
    printf("CM4 booted: %d\n", ret);
  }

  watchdog_init(IWDG_PRESCALER_16);

#ifdef PORTENTA_DEBUG_WIRED
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 1);
#endif

  while (1) {

    handle_data();

  }
}
