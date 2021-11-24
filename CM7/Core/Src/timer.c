#include "timer.h"
#include "peripherals.h"
#include "main.h"
#include "stm32h7xx_hal.h"

HRTIM_HandleTypeDef hhrtim;

static void MX_HRTIM_Init(void) {

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  hhrtim.Instance = HRTIM1;
  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim) != HAL_OK) {
    Error_Handler();
  }
  HAL_HRTIM_MspPostInit(&hhrtim);
}

void timer_init() {
  MX_HRTIM_Init();
}