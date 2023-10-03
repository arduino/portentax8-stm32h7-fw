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
#include "can_api.h"
#include "stm32h7xx_ll_hsem.h"
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

/* Special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

/* Valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

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

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

can_t fdcan_1;
can_t fdcan_2;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

/**
 * @brief FDCAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hfdcan->Instance == FDCAN1) {
    /* USER CODE BEGIN FDCAN1_MspInit 0 */

    /* USER CODE END FDCAN1_MspInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD1     ------> FDCAN1_TX
    PD0     ------> FDCAN1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */
  } else if (hfdcan->Instance == FDCAN2) {
    /* USER CODE BEGIN FDCAN2_MspInit 0 */

    /* USER CODE END FDCAN2_MspInit 0 */
    /* Peripheral clock enable */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    /* USER CODE BEGIN FDCAN2_MspInit 1 */

    /* USER CODE END FDCAN2_MspInit 1 */
  }
}

/**
 * @brief FDCAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan->Instance == FDCAN1) {
    /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

    /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PD1     ------> FDCAN1_TX
    PD0     ------> FDCAN1_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_1 | GPIO_PIN_0);

    /* FDCAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

    /* USER CODE END FDCAN1_MspDeInit 1 */
  } else if (hfdcan->Instance == FDCAN2) {
    /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

    /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

    /* FDCAN2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

    /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}


static void error(char* string) {
    printf(string);
    while (1);
}

void fdcan1_handler(uint8_t opcode, uint8_t *data, uint16_t size) {
    if (opcode == CONFIGURE)
    {
      uint32_t const can_bitrate = *((uint32_t *)data);
      can_frequency(&fdcan_1, can_bitrate);
      dbg_printf("fdcan1_handler: configuring fdcan1 with frequency %ld\n", can_bitrate);
    }
    else if (opcode == CAN_FILTER)
    {
      uint32_t const * info = (uint32_t*)data;

      uint32_t const handle = info[0];
      uint32_t const id     = info[1];
      uint32_t const mask   = info[2];

      CANFormat const format = (id < 0x800) ? CANStandard : CANExtended;

      if (!can_filter(&fdcan_1, id, mask, format, handle)) {
        dbg_printf("fdcan1_handler: can_filter failed for id: %ld, mask: %ld, format: %d, handle %ld\n", id, mask, format, handle);
      }
    }
    else if (opcode == CAN_TX_FRAME)
    {
      union x8h7_can_message msg;
      memcpy(&msg, data, size);

      dbg_printf("fdcan1_handler: sending CAN message to %x, size %d, content[0]=0x%02X\n", msg.id, msg.len, msg.data[0]);

      if (!can_write(&fdcan_1, &msg))
        can_reset(&fdcan_1);
    }
    else {
      dbg_printf("fdcan1_handler: error invalid opcode (:%d)\n", opcode);
    }
}

void fdcan2_handler(uint8_t opcode, uint8_t *data, uint16_t size) {
    if (opcode == CONFIGURE)
    {
      uint32_t const can_bitrate = *((uint32_t *)data);
      can_frequency(&fdcan_2, can_bitrate);
      dbg_printf("fdcan2_handler: configuring fdcan2 with frequency %ld\n", can_bitrate);
    }
    else if (opcode == CAN_FILTER)
    {
        uint32_t const * info = (uint32_t*)data;

        uint32_t const handle = info[0];
        uint32_t const id     = info[1];
        uint32_t const mask   = info[2];

        CANFormat const format = (id < 0x800) ? CANStandard : CANExtended;

        if (!can_filter(&fdcan_2, id, mask, format, handle)) {
          dbg_printf("fdcan2_handler: can_filter failed for id: %ld, mask: %ld, format: %d, handle %ld\n", id, mask, format, handle);
        }
    }
    else if (opcode == CAN_TX_FRAME)
    {
      union x8h7_can_message msg;
      memcpy(&msg, data, size);

      dbg_printf("fdcan2_handler: sending CAN message to %x, size %d, content[0]=0x%02X\n", msg.id, msg.len, msg.data[0]);

      if (!can_write(&fdcan_2, &msg))
        can_reset(&fdcan_2);
    }
    else {
      dbg_printf("fdcan2_handler: error invalid opcode (:%d)\n", opcode);
    }
}


void canInit()
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

  can_init(&fdcan_1, CAN_1, default_can_bit_timing);
  can_init(&fdcan_2, CAN_2, default_can_bit_timing);

  register_peripheral_callback(PERIPH_FDCAN1, &fdcan1_handler);
  register_peripheral_callback(PERIPH_FDCAN2, &fdcan2_handler);
}

void can_handle_data()
{
  union x8h7_can_message msg;

  if (can_read(&fdcan_1, &msg)) {
    enqueue_packet(PERIPH_FDCAN1, DATA, sizeof(msg.buf), msg.buf);
  }

  if (can_read(&fdcan_2, &msg)) {
    enqueue_packet(PERIPH_FDCAN2, DATA, sizeof(msg.buf), msg.buf);
  }
}

/** Call all the init functions
 *
 *  @returns
 *    0 if mode change failed or unsupported,
 *    1 if mode change was successful
 */
int can_internal_init(can_t *obj)
{
    if (HAL_FDCAN_Init(&obj->CanHandle) != HAL_OK) {
        error("HAL_FDCAN_Init error\n");
    }

    if (can_filter(obj, 0, 0, CANStandard, 0) == 0) {
        error("can_filter error\n");
    }

    if (can_filter(obj, 0, 0, CANExtended, 0) == 0) {
        error("can_filter error\n");
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&obj->CanHandle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        error("HAL_FDCAN_ConfigGlobalFilter error\n");
    }

    if (HAL_FDCAN_Start(&obj->CanHandle) != HAL_OK) {
        error("HAL_FDCAN_Start error\n");
    }

    return 1;
}

void can_init(can_t *obj, CANName peripheral, CanNominalBitTimingResult const can_bit_timing)
{
    __HAL_RCC_FDCAN_CLK_ENABLE();
    HAL_RCC_FDCAN_CLK_ENABLED++;

    if (peripheral == CAN_1) {
        obj->index = 0;
    }
    else if (peripheral == CAN_2) {
        obj->index = 1;
    }

    // Default values
    obj->CanHandle.Instance = (FDCAN_GlobalTypeDef *)peripheral;

    obj->CanHandle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    obj->CanHandle.Init.Mode = FDCAN_MODE_NORMAL;
    obj->CanHandle.Init.AutoRetransmission = ENABLE;
    obj->CanHandle.Init.TransmitPause = DISABLE;
    obj->CanHandle.Init.ProtocolException = ENABLE;
    obj->CanHandle.Init.NominalPrescaler = can_bit_timing.baud_rate_prescaler;
    obj->CanHandle.Init.NominalTimeSeg1 = can_bit_timing.time_segment_1;
    obj->CanHandle.Init.NominalTimeSeg2 = can_bit_timing.time_segment_2;
    obj->CanHandle.Init.NominalSyncJumpWidth = obj->CanHandle.Init.NominalTimeSeg2; // Synchronization_Jump_width
    obj->CanHandle.Init.DataPrescaler = 0x1;       // Not used - only in FDCAN
    obj->CanHandle.Init.DataSyncJumpWidth = 0x1;   // Not used - only in FDCAN
    obj->CanHandle.Init.DataTimeSeg1 = 0x1;        // Not used - only in FDCAN
    obj->CanHandle.Init.DataTimeSeg2 = 0x1;        // Not used - only in FDCAN

    /* Message RAM offset is only supported in STM32H7 platforms of supported FDCAN platforms */
    obj->CanHandle.Init.MessageRAMOffset = 0;

    /* The number of Standard and Extended ID filters are initialized to the maximum possile extent
     * for STM32H7 platforms
     */
    obj->CanHandle.Init.StdFiltersNbr = 128; // to be aligned with the handle parameter in can_filter
    obj->CanHandle.Init.ExtFiltersNbr = 64; // to be aligned with the handle parameter in can_filter

    obj->CanHandle.Init.RxFifo0ElmtsNbr = 8;
    obj->CanHandle.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    obj->CanHandle.Init.RxFifo1ElmtsNbr = 0;
    obj->CanHandle.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    obj->CanHandle.Init.RxBuffersNbr = 0;
    obj->CanHandle.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    obj->CanHandle.Init.TxEventsNbr = 3;
    obj->CanHandle.Init.TxBuffersNbr = 0;
    obj->CanHandle.Init.TxFifoQueueElmtsNbr = 3;

    obj->CanHandle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    obj->CanHandle.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    can_internal_init(obj);
}

/** Reset CAN interface.
 *
 * To use after error overflow.
 */
void can_reset(can_t *obj)
{
    can_mode(obj, MODE_RESET);
    HAL_FDCAN_ResetTimeoutCounter(&obj->CanHandle);
    HAL_FDCAN_ResetTimestampCounter(&obj->CanHandle);
}


int can_frequency(can_t *obj, uint32_t const can_bitrate)
{
    if (HAL_FDCAN_Stop(&obj->CanHandle) != HAL_OK) {
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

    obj->CanHandle.Init.NominalPrescaler = can_bit_timing.baud_rate_prescaler;
    obj->CanHandle.Init.NominalTimeSeg1 = can_bit_timing.time_segment_1;
    obj->CanHandle.Init.NominalTimeSeg2 = can_bit_timing.time_segment_2;
    obj->CanHandle.Init.NominalSyncJumpWidth = obj->CanHandle.Init.NominalTimeSeg2; // Synchronization_Jump_width

    return can_internal_init(obj);
}


/** Filter out incoming messages
 *
 *  @param obj CAN object
 *  @param id the id to filter on
 *  @param mask the mask applied to the id
 *  @param format format to filter on
 *  @param handle message filter handle (not supported yet)
 *
 *  @returns
 *    0 if filter change failed or unsupported,
 *    new filter handle if successful (not supported yet => returns 1)
 */
int can_filter(can_t *obj, uint32_t id, uint32_t mask, CANFormat format, int32_t handle)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    if (format == CANStandard) {
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = handle;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = id;
        sFilterConfig.FilterID2 = mask;
    } else if (format == CANExtended) {
        sFilterConfig.IdType = FDCAN_EXTENDED_ID;
        sFilterConfig.FilterIndex = handle;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = id;
        sFilterConfig.FilterID2 = mask;
    } else { // Filter for CANAny format cannot be configured for STM32
        return 0;
    }

    if (HAL_FDCAN_ConfigFilter(&obj->CanHandle, &sFilterConfig) != HAL_OK) {
        return 0;
    }

    return 1;
}


int can_write(can_t *obj, union x8h7_can_message const * msg)
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

    if (HAL_FDCAN_AddMessageToTxFifoQ(&obj->CanHandle, &TxHeader, (uint8_t *)msg->field.data) != HAL_OK)
    {
      uint32_t const err_code = HAL_FDCAN_GetError(&obj->CanHandle);
      printf("error: %ld\n", err_code);

      uint8_t msg[2] = {X8H7_CAN_STS_INT_ERR, 0};
      if (err_code == HAL_FDCAN_ERROR_FIFO_FULL) msg[1] = X8H7_CAN_STS_FLG_TX_OVR;

      enqueue_packet(obj == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(msg), msg);
      return 0;
    }
    else
    {
      uint8_t msg[2] = {X8H7_CAN_STS_INT_TX, 0};
      enqueue_packet(obj == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(msg), msg);
      return 1;
    }
}

int can_read(can_t *obj, union x8h7_can_message *msg)
{
  static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    if (HAL_FDCAN_GetRxFifoFillLevel(&obj->CanHandle, FDCAN_RX_FIFO0) == 0) {
        return 0; // No message arrived
    }

  FDCAN_RxHeaderTypeDef RxHeader = {0};
  uint8_t RxData[64] = {0};
  if (HAL_FDCAN_GetRxMessage(&obj->CanHandle, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
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

unsigned char can_rderror(can_t *obj)
{
    FDCAN_ErrorCountersTypeDef ErrorCounters;

    HAL_FDCAN_GetErrorCounters(&obj->CanHandle, &ErrorCounters);

    return (unsigned char)ErrorCounters.RxErrorCnt;
}

unsigned char can_tderror(can_t *obj)
{
    FDCAN_ErrorCountersTypeDef ErrorCounters;

    HAL_FDCAN_GetErrorCounters(&obj->CanHandle, &ErrorCounters);

    return (unsigned char)ErrorCounters.TxErrorCnt;
}

/** Change CAN operation to the specified mode
 *
 *  @param mode The new operation mode (MODE_RESET, MODE_NORMAL, MODE_SILENT, MODE_TEST_LOCAL, MODE_TEST_GLOBAL, MODE_TEST_SILENT)
 *
 *  @returns
 *    0 if mode change failed or unsupported,
 *    1 if mode change was successful
 */
int can_mode(can_t *obj, CanMode mode)
{
    if (HAL_FDCAN_Stop(&obj->CanHandle) != HAL_OK) {
        error("HAL_FDCAN_Stop error\n");
    }

    switch (mode) {
        case MODE_RESET:
            break;
        case MODE_NORMAL:
            obj->CanHandle.Init.Mode = FDCAN_MODE_NORMAL;
            // obj->CanHandle.Init.NominalPrescaler = 100;   // Prescaler
            break;
        case MODE_SILENT: // Bus Monitoring
            obj->CanHandle.Init.Mode = FDCAN_MODE_BUS_MONITORING;
            break;
        case MODE_TEST_GLOBAL: // External LoopBack
        case MODE_TEST_LOCAL:
            obj->CanHandle.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
            break;
        case MODE_TEST_SILENT: // Internal LoopBack
            obj->CanHandle.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
            // obj->CanHandle.Init.NominalPrescaler = 1;   // Prescaler
            break;
        default:
            return 0;
    }

    return can_internal_init(obj);
}
