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

#include "spi.h"
#include "error_handler.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

/**************************************************************************************
 * GLOBAL VARIABLE
 **************************************************************************************/

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

/**************************************************************************************
 * FUNCTION DEFINIION
 **************************************************************************************/

static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Peripheral clock enable */
  __HAL_RCC_SPI3_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /**SPI3 GPIO Configuration
  PA15 (JTDI)     ------> SPI3_NSS
  PC10     ------> SPI3_SCK
  PC12     ------> SPI3_MOSI
  PC11     ------> SPI3_MISO
  */

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 DMA Init */
  /* SPI3_TX Init */
  hdma_spi3_tx.Instance = DMA1_Stream0;
  hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
  hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi3_tx.Init.Mode = DMA_NORMAL;
  hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);

  /* SPI3_RX Init */
  hdma_spi3_rx.Instance = DMA1_Stream1;
  hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
  hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi3_rx.Init.Mode = DMA_NORMAL;
  hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

  /* SPI3 interrupt Init */
  HAL_NVIC_SetPriority(SPI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  __HAL_RCC_SPI3_CLK_DISABLE();

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_11);

  HAL_DMA_DeInit(hspi->hdmatx);
  HAL_DMA_DeInit(hspi->hdmarx);

  HAL_NVIC_DisableIRQ(SPI3_IRQn);
}

void spi_init()
{
  MX_SPI3_Init();
}

void spi_end()
{
  HAL_SPI_Abort(&hspi3);
}

void spi_transmit_receive(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t size)
{
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buf, rx_buf, size);
}
