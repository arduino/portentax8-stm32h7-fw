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

#include "adc.h"
#include "main.h"
#include "debug.h"
#include "system.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

struct ADC_numbers {
  ADC_HandleTypeDef* peripheral;
  uint32_t channel;
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

struct ADC_numbers ADC_pinmap[] = {
  { NULL, 0 },
  { &hadc1, ADC_CHANNEL_2 },
  { &hadc2, ADC_CHANNEL_3 },
  { &hadc2, ADC_CHANNEL_2 },
  { &hadc2, ADC_CHANNEL_5 },
  { &hadc2, ADC_CHANNEL_4 },
  { &hadc3, ADC_CHANNEL_3 },
  { &hadc3, ADC_CHANNEL_2 },
  { &hadc3, ADC_CHANNEL_4 },
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void adc_init() {

  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
}

int get_adc_value(enum AnalogPins name) {
  ADC_ChannelConfTypeDef conf = {0};
  ADC_HandleTypeDef* peripheral;

  conf.Rank = ADC_REGULAR_RANK_1;
  conf.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  conf.SingleDiff = ADC_SINGLE_ENDED;
  conf.OffsetNumber = ADC_OFFSET_NONE;

  conf.Channel = ADC_pinmap[name].channel;
  peripheral = ADC_pinmap[name].peripheral;

  HAL_ADC_ConfigChannel(peripheral, &conf);
  HAL_ADC_Start(peripheral);
  HAL_ADC_PollForConversion(peripheral, 10);
  uint16_t value = HAL_ADC_GetValue(peripheral);
  HAL_ADC_Stop(peripheral);

  dbg_printf("ADC%d: %d\n", name-1, value);

  return enqueue_packet(PERIPH_ADC, name, sizeof(value), &value);
}

static void MX_ADC1_Init(void) {

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.State = HAL_ADC_STATE_RESET;
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

static void MX_ADC2_Init(void) {

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.State = HAL_ADC_STATE_RESET;
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

static void MX_ADC3_Init(void) {

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc3.State = HAL_ADC_STATE_RESET;
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hadc->Instance == ADC1) {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PF11     ------> ADC1_INP2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  } else if (hadc->Instance == ADC2) {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PB1     ------> ADC2_INP5
    PC4     ------> ADC2_INP4
    PA6     ------> ADC2_INP3
    PF13     ------> ADC2_INP2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC2_MspInit 1 */

    /* USER CODE END ADC2_MspInit 1 */
  } else if (hadc->Instance == ADC3) {
    /* USER CODE BEGIN ADC3_MspInit 0 */

    /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PF5     ------> ADC3_INP4
    PF7     ------> ADC3_INP3
    PF9     ------> ADC3_INP2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC3_MspInit 1 */

    /* USER CODE END ADC3_MspInit 1 */
  }
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PF11     ------> ADC1_INP2
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_11);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  } else if (hadc->Instance == ADC2) {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if (HAL_RCC_ADC12_CLK_ENABLED == 0) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PB1     ------> ADC2_INP5
    PC4     ------> ADC2_INP4
    PA6     ------> ADC2_INP3
    PF13     ------> ADC2_INP2
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_13);

    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  } else if (hadc->Instance == ADC3) {
    /* USER CODE BEGIN ADC3_MspDeInit 0 */

    /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PF5     ------> ADC3_INP4
    PF7     ------> ADC3_INP3
    PF9     ------> ADC3_INP2
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_9);

    /* USER CODE BEGIN ADC3_MspDeInit 1 */

    /* USER CODE END ADC3_MspDeInit 1 */
  }
}