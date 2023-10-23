/* mbed Microcontroller Library
 * Copyright (c) 2006-2017 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "can.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_hsem.h"
#include "debug.h"
#include "system.h"
#include "peripherals.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define TARGET_STM32H7      1
#define CFG_HW_RCC_SEMID    3
#undef DUAL_CORE

#define X8H7_CAN_STS_FLG_RX_OVR  0x01  // Receive Buffer Overflow
#define X8H7_CAN_STS_FLG_TX_BO   0x02  // Bus-Off
#define X8H7_CAN_STS_FLG_TX_EP   0x04  // Transmit Error-Passive
#define X8H7_CAN_STS_FLG_RX_EP   0x08  // Receive Error-Passive
#define X8H7_CAN_STS_FLG_TX_WAR  0x10  // Transmit Error Warning
#define X8H7_CAN_STS_FLG_RX_WAR  0x20  // Receive Error Warning
#define X8H7_CAN_STS_FLG_EWARN   0x40  // Error Warning
#define X8H7_CAN_STS_FLG_TX_OVR  0x80  // Transmit Buffer Overflow

#define X8H7_CAN_STS_INT_TX      0x01
#define X8H7_CAN_STS_INT_RX      0x02
#define X8H7_CAN_STS_INT_ERR     0x04

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint32_t const TQ_MIN    =   1;
static uint32_t const TQ_MAX    = 512;
static uint32_t const TSEG1_MIN =   1;
static uint32_t const TSEG1_MAX = 256;
static uint32_t const TSEG2_MIN =   1;
static uint32_t const TSEG2_MAX = 128;

static uint32_t const DEFAULT_CAN_BIT_RATE = 100*1000UL; /* 100 kBit/s */

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FDCAN_HandleTypeDef fdcan_1;
FDCAN_HandleTypeDef fdcan_2;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hfdcan->Instance == FDCAN1)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /*
     * FDCAN1 GPIO Configuration
     * PD1     ------> FDCAN1_TX
     * PD0     ------> FDCAN1_RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
  else if (hfdcan->Instance == FDCAN2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*
     * FDCAN2 GPIO Configuration
     * PB5     ------> FDCAN2_RX
     * PB6     ------> FDCAN2_TX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN1)
  {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_1 | GPIO_PIN_0);
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
  else if (hfdcan->Instance == FDCAN2)
  {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  }
}

static void error(char* string) {
    printf(string);
    while (1);
}

void can_init()
{
  CanNominalBitTimingResult default_can_bit_timing = {0};

  if (!calc_can_nominal_bit_timing(DEFAULT_CAN_BIT_RATE,
                                   HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN),
                                   TQ_MAX,
                                   TQ_MIN,
                                   TSEG1_MIN,
                                   TSEG1_MAX,
                                   TSEG2_MIN,
                                   TSEG2_MAX,
                                   &default_can_bit_timing))
  {
    printf("Could not calculate valid default CAN bit timing\n");
    return;
  }

  can_init_device(&fdcan_1, CAN_1, default_can_bit_timing);
  can_init_device(&fdcan_2, CAN_2, default_can_bit_timing);
}

int can_handle_data()
{
  int bytes_enqueued = 0;
  union x8h7_can_frame_message msg;

  /* Note: the last read package is lost in this implementation. We need to fix this by
   * implementing some peek method or by buffering messages in a ringbuffer.
   */

  for (int rc_enq = 0; can_read(&fdcan_1, &msg); bytes_enqueued += rc_enq)
  {
    rc_enq = enqueue_packet(PERIPH_FDCAN1, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + msg.field.len, msg.buf);
    if (!rc_enq) return bytes_enqueued;
  }

  for (int rc_enq = 0; can_read(&fdcan_2, &msg); bytes_enqueued += rc_enq)
  {
    rc_enq = enqueue_packet(PERIPH_FDCAN2, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + msg.field.len, msg.buf);
    if (!rc_enq) return bytes_enqueued;
  }

  return bytes_enqueued;
}

/** Call all the init functions
 *
 *  @returns
 *    0 if mode change failed or unsupported,
 *    1 if mode change was successful
 */
int can_internal_init(FDCAN_HandleTypeDef * handle)
{
    if (HAL_FDCAN_Init(handle) != HAL_OK) {
        error("HAL_FDCAN_Init error\n");
    }

    if (can_filter(handle, 0, 0, 0, false) == 0) {
        error("can_filter error\n");
    }

    if (can_filter(handle, 0, 0, 0, true) == 0) {
        error("can_filter error\n");
    }

    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        error("HAL_FDCAN_ConfigGlobalFilter error\n");
    }

    if (HAL_FDCAN_Start(handle) != HAL_OK) {
        error("HAL_FDCAN_Start error\n");
    }

    return 1;
}

void can_init_device(FDCAN_HandleTypeDef * handle, CANName peripheral, CanNominalBitTimingResult const can_bit_timing)
{
    __HAL_RCC_FDCAN_CLK_ENABLE();
    HAL_RCC_FDCAN_CLK_ENABLED++;

    // Default values
    handle->Instance = (FDCAN_GlobalTypeDef *)peripheral;

    handle->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    handle->Init.Mode = FDCAN_MODE_NORMAL;
    handle->Init.AutoRetransmission = ENABLE;
    handle->Init.TransmitPause = DISABLE;
    handle->Init.ProtocolException = ENABLE;
    handle->Init.NominalPrescaler = can_bit_timing.baud_rate_prescaler;
    handle->Init.NominalTimeSeg1 = can_bit_timing.time_segment_1;
    handle->Init.NominalTimeSeg2 = can_bit_timing.time_segment_2;
    handle->Init.NominalSyncJumpWidth = handle->Init.NominalTimeSeg2; // Synchronization_Jump_width
    handle->Init.DataPrescaler = 0x1;       // Not used - only in FDCAN
    handle->Init.DataSyncJumpWidth = 0x1;   // Not used - only in FDCAN
    handle->Init.DataTimeSeg1 = 0x1;        // Not used - only in FDCAN
    handle->Init.DataTimeSeg2 = 0x1;        // Not used - only in FDCAN

    /* Message RAM offset is only supported in STM32H7 platforms of supported FDCAN platforms */
    handle->Init.MessageRAMOffset = 0;

    /* The number of Standard and Extended ID filters are initialized to the maximum possible extent
     * for STM32H7 platforms
     */
    handle->Init.StdFiltersNbr = 128; // to be aligned with the handle parameter in can_filter
    handle->Init.ExtFiltersNbr = 64; // to be aligned with the handle parameter in can_filter

    handle->Init.RxFifo0ElmtsNbr = 8;
    handle->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    handle->Init.RxFifo1ElmtsNbr = 0;
    handle->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    handle->Init.RxBuffersNbr = 0;
    handle->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    handle->Init.TxEventsNbr = 3;
    handle->Init.TxBuffersNbr = 0;
    handle->Init.TxFifoQueueElmtsNbr = 3;

    handle->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    handle->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    can_internal_init(handle);
}

int can_frequency(FDCAN_HandleTypeDef * handle, uint32_t const can_bitrate)
{
    if (HAL_FDCAN_Stop(handle) != HAL_OK) {
        error("HAL_FDCAN_Stop error\n");
    }

  uint32_t const can_clock_Hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

  CanNominalBitTimingResult can_bit_timing = {0};

  if (!calc_can_nominal_bit_timing(can_bitrate,
                                   can_clock_Hz,
                                   TQ_MAX,
                                   TQ_MIN,
                                   TSEG1_MIN,
                                   TSEG1_MAX,
                                   TSEG2_MIN,
                                   TSEG2_MAX,
                                   &can_bit_timing))
  {
    printf("Could not calculate valid CAN bit timing\n");
    return 0;
  }

  dbg_printf("can_frequency:\n\r  can_bitrate = %ld\n\r  can_clock_Hz = %ld\n", can_bitrate, can_clock_Hz);

  handle->Init.NominalPrescaler = can_bit_timing.baud_rate_prescaler;
  handle->Init.NominalTimeSeg1 = can_bit_timing.time_segment_1;
  handle->Init.NominalTimeSeg2 = can_bit_timing.time_segment_2;
  handle->Init.NominalSyncJumpWidth = handle->Init.NominalTimeSeg2; // Synchronization_Jump_width

  return can_internal_init(handle);
}


int can_filter(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask, bool const is_extended_id)
{
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.IdType = is_extended_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = filter_index;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = is_extended_id ? (id & CAN_EFF_MASK) : (id & CAN_SFF_MASK);
  sFilterConfig.FilterID2 = is_extended_id ? (mask & CAN_EFF_MASK) : (mask & CAN_SFF_MASK);

  if (HAL_FDCAN_ConfigFilter(handle, &sFilterConfig) != HAL_OK) {
    return 0;
  }

  return 1;
}


int can_write(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};

  if (msg->field.id & CAN_EFF_FLAG)
  {
    TxHeader.IdType     = FDCAN_EXTENDED_ID;
    TxHeader.Identifier = msg->field.id & CAN_EFF_MASK;
  }
  else
  {
    TxHeader.IdType     = FDCAN_STANDARD_ID;
    TxHeader.Identifier = msg->field.id & CAN_SFF_MASK;
  }

    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    switch (msg->field.len)
    {
      default:
      case 0:  TxHeader.DataLength = FDCAN_DLC_BYTES_0;  break;
      case 1:  TxHeader.DataLength = FDCAN_DLC_BYTES_1;  break;
      case 2:  TxHeader.DataLength = FDCAN_DLC_BYTES_2;  break;
      case 3:  TxHeader.DataLength = FDCAN_DLC_BYTES_3;  break;
      case 4:  TxHeader.DataLength = FDCAN_DLC_BYTES_4;  break;
      case 5:  TxHeader.DataLength = FDCAN_DLC_BYTES_5;  break;
      case 6:  TxHeader.DataLength = FDCAN_DLC_BYTES_6;  break;
      case 7:  TxHeader.DataLength = FDCAN_DLC_BYTES_7;  break;
      case 8:  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  break;
      case 12: TxHeader.DataLength = FDCAN_DLC_BYTES_12; break;
      case 16: TxHeader.DataLength = FDCAN_DLC_BYTES_16; break;
      case 20: TxHeader.DataLength = FDCAN_DLC_BYTES_20; break;
      case 24: TxHeader.DataLength = FDCAN_DLC_BYTES_24; break;
      case 32: TxHeader.DataLength = FDCAN_DLC_BYTES_32; break;
      case 48: TxHeader.DataLength = FDCAN_DLC_BYTES_48; break;
      case 64: TxHeader.DataLength = FDCAN_DLC_BYTES_64; break;
    }
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &TxHeader, (uint8_t *)msg->field.data) != HAL_OK)
    {
      uint32_t const err_code = HAL_FDCAN_GetError(handle);
      printf("error: %ld\n", err_code);

      uint8_t msg[2] = {X8H7_CAN_STS_INT_ERR, 0};
      if (err_code == HAL_FDCAN_ERROR_FIFO_FULL) msg[1] = X8H7_CAN_STS_FLG_TX_OVR;

      return enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(msg), msg);
    }
    else
    {
      uint8_t msg[2] = {X8H7_CAN_STS_INT_TX, 0};
      return enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(msg), msg);
    }
}

int can_read(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message *msg)
{
  static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

  if (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) == 0)
    return 0; // No message arrived

  FDCAN_RxHeaderTypeDef RxHeader = {0};
  uint8_t RxData[64] = {0};
  if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    error("HAL_FDCAN_GetRxMessage error\n"); // Should not occur as previous HAL_FDCAN_GetRxFifoFillLevel call reported some data
    return 0;
  }

  if (RxHeader.IdType == FDCAN_EXTENDED_ID)
    msg->field.id = CAN_EFF_FLAG | (RxHeader.Identifier & CAN_EFF_MASK);
  else
    msg->field.id =                (RxHeader.Identifier & CAN_SFF_MASK);

  if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
    msg->field.id |= CAN_RTR_FLAG;

  msg->field.len = DLCtoBytes[RxHeader.DataLength >> 16];
  if (msg->field.len > sizeof(msg->field.data))
    msg->field.len = sizeof(msg->field.data);

  memcpy(msg->field.data, RxData, msg->field.len);

  return 1;
}

unsigned char can_rderror(FDCAN_HandleTypeDef * handle)
{
  FDCAN_ErrorCountersTypeDef ErrorCounters;
  HAL_FDCAN_GetErrorCounters(handle, &ErrorCounters);
  return (unsigned char)ErrorCounters.RxErrorCnt;
}

unsigned char can_tderror(FDCAN_HandleTypeDef * handle)
{
  FDCAN_ErrorCountersTypeDef ErrorCounters;
  HAL_FDCAN_GetErrorCounters(handle, &ErrorCounters);
  return (unsigned char)ErrorCounters.TxErrorCnt;
}
