#include "timer.h"
#include "system.h"
#include "peripherals.h"
#include "main.h"

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

void pwm_timer_config(uint32_t index, uint32_t channel,
                      HRTIM_SimplePWMChannelCfgTypeDef* pSimplePWMChannelCfg,
                      HRTIM_TimeBaseCfgTypeDef * pTimeBaseCfg,
                      uint32_t timers, bool enable) {

  HAL_HRTIM_TimeBaseConfig(&hhrtim, index, pTimeBaseCfg);
  HAL_HRTIM_SimplePWMChannelConfig(&hhrtim, index, channel, pSimplePWMChannelCfg);
  HAL_HRTIM_SoftwareUpdate(&hhrtim, timers);

  if (enable) {
    HAL_HRTIM_SimplePWMStart(&hhrtim, index, channel);
  } else {
    HAL_HRTIM_SimplePWMStop(&hhrtim, index, channel);
  }

}