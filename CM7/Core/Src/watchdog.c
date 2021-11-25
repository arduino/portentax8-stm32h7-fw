#include "watchdog.h"
#include "main.h"
#include "stm32h7xx_hal.h"

IWDG_HandleTypeDef watchdog;

void watchdog_init(int prescaler) {
  watchdog.Instance = IWDG1;
  watchdog.Init.Prescaler = prescaler;
  watchdog.Init.Reload = (32000 * 2000) / (16 * 1000); /* 2000 ms */
  watchdog.Init.Window = (32000 * 2000) / (16 * 1000); /* 2000 ms */

  HAL_IWDG_Init(&watchdog);
}

void watchdog_refresh() {
  HAL_IWDG_Refresh(&watchdog);
}