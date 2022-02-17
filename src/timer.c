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

#include "timer.h"
#include "system.h"
#include "peripherals.h"
#include "main.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

HRTIM_HandleTypeDef hhrtim;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

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

/**
 * @brief HRTIM MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hhrtim: HRTIM handle pointer
 * @retval None
 */
void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef *hhrtim) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (hhrtim->Instance == HRTIM1) {
    /* USER CODE BEGIN HRTIM1_MspInit 0 */

    /* USER CODE END HRTIM1_MspInit 0 */
    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1;
    PeriphClkInitStruct.Hrtim1ClockSelection = RCC_HRTIM1CLK_TIMCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();
    /* USER CODE BEGIN HRTIM1_MspInit 1 */

    /* USER CODE END HRTIM1_MspInit 1 */
  }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hhrtim->Instance == HRTIM1) {
    /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

    /* USER CODE END HRTIM1_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**HRTIM GPIO Configuration
    PA10     ------> HRTIM_CHC2
    PA8     ------> HRTIM_CHB2
    PA9     ------> HRTIM_CHC1
    PA11     ------> HRTIM_CHD1
    PA12     ------> HRTIM_CHD2
    PC7     ------> HRTIM_CHA2
    PG7     ------> HRTIM_CHE2
    PC8     ------> HRTIM_CHB1
    PC6     ------> HRTIM_CHA1
    PG6     ------> HRTIM_CHE1
    */
    GPIO_InitStruct.Pin =
        GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_HRTIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_HRTIM1;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

    /* USER CODE END HRTIM1_MspPostInit 1 */
  }
}
/**
 * @brief HRTIM MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hhrtim: HRTIM handle pointer
 * @retval None
 */
void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef *hhrtim) {
  if (hhrtim->Instance == HRTIM1) {
    /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

    /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();
    /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

    /* USER CODE END HRTIM1_MspDeInit 1 */
  }
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