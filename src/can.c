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

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

can_t fdcan_1;
can_t fdcan_2;

static uintptr_t can_irq_contexts[2] = {0};
static can_irq_handler irq_handler;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

#if 0
static void MX_FDCAN1_Init(void) {

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_FDCAN2_Init(void) {

  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}
#endif

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
    /* Peripheral clock enable */

    /*
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }
    */

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
    #if 0
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }
    #endif

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
    if (opcode == CONFIGURE) {
        configureFDCAN(PERIPH_FDCAN1, data);
    } else if (opcode == CAN_FILTER) {
        uint32_t* info = (uint32_t*)data;
        CANFormat format = info[1] < 0x800 ? CANStandard : CANExtended;
        canFilter(PERIPH_FDCAN1, info[1], info[2], format, info[0]);
    } else if (opcode == CAN_TX_FRAME) {
        CAN_Message msg;
        msg.type = CANData;
        msg.format = CANStandard;
        memcpy(&msg, data, size);

        dbg_printf("Sending CAN message to %x, size %d, content[0]=0x%02X\n",
        msg.id, msg.len, msg.data[0]);

        if (msg.id > 0x7FF) {
        msg.format = CANExtended;
        }

        if (!can_write(&fdcan_1, msg))
        {
          can_reset(&fdcan_1);
        }
    } else {
      dbg_printf("fdcan1_handler: error invalid opcode (:%d)\n", opcode);
    }
}

void fdcan2_handler(uint8_t opcode, uint8_t *data, uint16_t size) {
    if (opcode == CONFIGURE) {
        configureFDCAN(PERIPH_FDCAN2, data);
    } else if (opcode == CAN_FILTER) {
        uint32_t* info = (uint32_t*)data;
        CANFormat format = info[1] < 0x800 ? CANStandard : CANExtended;
        canFilter(PERIPH_FDCAN2, info[1], info[2], format, info[0]);
    } else if (opcode == CAN_TX_FRAME) {
        CAN_Message msg;
        msg.type = CANData;
        msg.format = CANStandard;
        memcpy(&msg, data, size);

        dbg_printf("Sending CAN message to %x, size %d, content[0]=0x%02X\n",
        msg.id, msg.len, msg.data[0]);

        if (msg.id > 0x7FF) {
        msg.format = CANExtended;
        }

        if (!can_write(&fdcan_2, msg))
        {
          can_reset(&fdcan_2);
        }
    } else {
      dbg_printf("fdcan2_handler: error invalid opcode (:%d)\n", opcode);
    }
}


void canInit()
{
    can_init_freq_direct(&fdcan_1, CAN_1, 800000);
    can_init_freq_direct(&fdcan_2, CAN_2, 800000);

    register_peripheral_callback(PERIPH_FDCAN1, &fdcan1_handler);
    register_peripheral_callback(PERIPH_FDCAN2, &fdcan2_handler);

/*
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  */
}

int canFilter(uint8_t peripheral, uint32_t id, uint32_t mask, CANFormat format, int32_t handle)
{
    if (peripheral == PERIPH_FDCAN1) {
        return can_filter(&fdcan_1, id, mask, format, handle);
    }
    if (peripheral == PERIPH_FDCAN2) {
        return can_filter(&fdcan_2, id, mask, format, handle);
    }
    return 0;
}

void can_handle_data() {
    CAN_Message msg;
    if (can_read(&fdcan_1, &msg)) {
      enqueue_packet(PERIPH_FDCAN1, DATA, sizeof(msg), &msg);
    }

    if (can_read(&fdcan_2, &msg)) {
      enqueue_packet(PERIPH_FDCAN2, DATA, sizeof(msg), &msg);
    }
}

void configureFDCAN(uint8_t peripheral, void* data) {

  if (peripheral == PERIPH_FDCAN1) {
    can_frequency(&fdcan_1, *((uint32_t*)data));
  } else {
    can_frequency(&fdcan_2, *((uint32_t*)data));
  }

  dbg_printf("Configuring fdcan%d with frequency %ld\n", peripheral == PERIPH_FDCAN1 ? 1 : 2, *((uint32_t*)data));

  //HAL_FDCAN_ConfigFilter(&_hfdcan1, filterDef);
  //HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt);
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

void can_init_freq_direct(can_t *obj, CANName peripheral, int hz)
{
#if defined(__HAL_RCC_FDCAN1_CLK_ENABLE)
    __HAL_RCC_FDCAN1_CLK_ENABLE();
#else
    __HAL_RCC_FDCAN_CLK_ENABLE();
#endif

    if (peripheral == CAN_1) {
        obj->index = 0;
    }
#if defined(FDCAN2_BASE)
    else if (peripheral == CAN_2) {
        obj->index = 1;
    }
#endif

#if 0
    // Select PLL2Q as source of FDCAN clock
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
#if (defined RCC_PERIPHCLK_FDCAN1)
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN1;
    RCC_PeriphClkInit.Fdcan1ClockSelection = RCC_FDCAN1CLKSOURCE_PLL1;
#else
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    RCC_PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
#endif
#if defined(DUAL_CORE) && (TARGET_STM32H7)
    while (LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID)) {
    }
#endif /* DUAL_CORE */
    if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit) != HAL_OK) {
        error("HAL_RCCEx_PeriphCLKConfig error\n");
    }
#if defined(DUAL_CORE) && (TARGET_STM32H7)
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, HSEM_CR_COREID_CURRENT);
#endif /* DUAL_CORE */

#endif

    // Default values
    obj->CanHandle.Instance = (FDCAN_GlobalTypeDef *)peripheral;

    /* Bit time parameter
                                ex with 100 kHz   requested frequency hz
    fdcan_ker_ck               | 10 MHz         | 10 MHz
    Prescaler                  | 1              | 1
    Time_quantum (tq)          | 100 ns         | 100 ns
    Bit_rate                   | 0.1 MBit/s     | <hz>
    Bit_length                 | 10 µs = 100 tq | <n_tq> = 10 000 000 / <hz>
    Synchronization_segment    | 1 tq           | 1 tq
    Phase_segment_1            | 69 tq          | <nts1> = <n_tq> * 0.75
    Phase_segment_2            | 30 tq          | <nts2> = <n_tq> - 1 - <nts1>
    Synchronization_Jump_width | 30 tq          | <nsjw> = <nts2>
    */

#if (defined RCC_PERIPHCLK_FDCAN1)
    int ntq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN1) / hz;
#else
    int ntq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / hz;
#endif

    int nominalPrescaler = 1;
    // !When the sample point should be lower than 50%, this must be changed to
    // !IS_FDCAN_NOMINAL_TSEG2(ntq/nominalPrescaler), since
    // NTSEG2 and SJW max values are lower. For now the sample point is fix @75%
    while (!IS_FDCAN_NOMINAL_TSEG1(ntq / nominalPrescaler)) {
        nominalPrescaler ++;
        if (!IS_FDCAN_NOMINAL_PRESCALER(nominalPrescaler)) {
            error("Could not determine good nominalPrescaler. Bad clock value\n");
        }
    }
    ntq = ntq / nominalPrescaler;

    obj->CanHandle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    obj->CanHandle.Init.Mode = FDCAN_MODE_NORMAL;
    obj->CanHandle.Init.AutoRetransmission = ENABLE;
    obj->CanHandle.Init.TransmitPause = DISABLE;
    obj->CanHandle.Init.ProtocolException = ENABLE;
    obj->CanHandle.Init.NominalPrescaler = nominalPrescaler;      // Prescaler
    obj->CanHandle.Init.NominalTimeSeg1 = ntq * 0.75;      // Phase_segment_1
    obj->CanHandle.Init.NominalTimeSeg2 = ntq - 1 - obj->CanHandle.Init.NominalTimeSeg1;      // Phase_segment_2
    obj->CanHandle.Init.NominalSyncJumpWidth = obj->CanHandle.Init.NominalTimeSeg2; // Synchronization_Jump_width
    obj->CanHandle.Init.DataPrescaler = 0x1;       // Not used - only in FDCAN
    obj->CanHandle.Init.DataSyncJumpWidth = 0x1;   // Not used - only in FDCAN
    obj->CanHandle.Init.DataTimeSeg1 = 0x1;        // Not used - only in FDCAN
    obj->CanHandle.Init.DataTimeSeg2 = 0x1;        // Not used - only in FDCAN
#ifdef TARGET_STM32H7
    /* Message RAM offset is only supported in STM32H7 platforms of supported FDCAN platforms */
    obj->CanHandle.Init.MessageRAMOffset = 0;

    /* The number of Standard and Extended ID filters are initialized to the maximum possile extent
     * for STM32H7 platforms
     */
    obj->CanHandle.Init.StdFiltersNbr = 128; // to be aligned with the handle parameter in can_filter
    obj->CanHandle.Init.ExtFiltersNbr = 64; // to be aligned with the handle parameter in can_filter
#else
    /* The number of Standard and Extended ID filters are initialized to the maximum possile extent 
     * for STM32G0x1, STM32G4 and STM32L5  platforms
    */
    obj->CanHandle.Init.StdFiltersNbr = 28; // to be aligned with the handle parameter in can_filter
    obj->CanHandle.Init.ExtFiltersNbr = 8; // to be aligned with the handle parameter in can_filter
#endif
#ifdef TARGET_STM32H7
    obj->CanHandle.Init.RxFifo0ElmtsNbr = 8;
    obj->CanHandle.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    obj->CanHandle.Init.RxFifo1ElmtsNbr = 0;
    obj->CanHandle.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    obj->CanHandle.Init.RxBuffersNbr = 0;
    obj->CanHandle.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    obj->CanHandle.Init.TxEventsNbr = 3;
    obj->CanHandle.Init.TxBuffersNbr = 0;
    obj->CanHandle.Init.TxFifoQueueElmtsNbr = 3;
#endif
    obj->CanHandle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
#ifdef TARGET_STM32H7
    obj->CanHandle.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
#endif

    can_internal_init(obj);
}

void can_irq_init(can_t *obj, can_irq_handler handler, uintptr_t context)
{
    irq_handler = handler;
    can_irq_contexts[obj->index] = context;
}

void can_irq_free(can_t *obj)
{
    CANName can = (CANName)obj->CanHandle.Instance;
    if (can == CAN_1) {
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
    }
#if defined(FDCAN2_BASE)
    else if (can == CAN_2) {
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
    }
#endif
#if defined(FDCAN3_BASE)
    else if (can == CAN_3) {
        HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN3_IT1_IRQn);
    }
#endif
    else {
        return;
    }
#if (defined TARGET_STM32H7)
    HAL_NVIC_DisableIRQ(FDCAN_CAL_IRQn);
#endif
    can_irq_contexts[obj->index] = 0;
}

void can_free(can_t *obj)
{
#if defined(DUAL_CORE) && (TARGET_STM32H7)
    while (LL_HSEM_1StepLock(HSEM, CFG_HW_RCC_SEMID)) {
    }
#endif /* DUAL_CORE */
#if defined(__HAL_RCC_FDCAN1_FORCE_RESET)
    __HAL_RCC_FDCAN1_FORCE_RESET();
    __HAL_RCC_FDCAN1_RELEASE_RESET();
#else
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();
#endif
#if defined(DUAL_CORE) && (TARGET_STM32H7)
    LL_HSEM_ReleaseLock(HSEM, CFG_HW_RCC_SEMID, HSEM_CR_COREID_CURRENT);
#endif /* DUAL_CORE */
#if defined(__HAL_RCC_FDCAN1_CLK_DISABLE)
    __HAL_RCC_FDCAN1_CLK_DISABLE();
#else
    __HAL_RCC_FDCAN_CLK_DISABLE();
#endif
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


int can_frequency(can_t *obj, int f)
{
    if (HAL_FDCAN_Stop(&obj->CanHandle) != HAL_OK) {
        error("HAL_FDCAN_Stop error\n");
    }


    /* See can_init_freq function for calculation details
     *
     * !Attention Not all bitrates can be covered with all fdcan-core-clk values. When a clk
     * does not work for the desired bitrate, change system_clock settings for FDCAN_CLK
     * (default FDCAN_CLK is PLLQ)
     */

#if (defined RCC_PERIPHCLK_FDCAN1)
    int ntq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN1) / f;
#else
    int ntq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / f;
#endif

    int nominalPrescaler = 1;
    // !When the sample point should be lower than 50%, this must be changed to
    // !IS_FDCAN_DATA_TSEG2(ntq/nominalPrescaler), since
    // NTSEG2 and SJW max values are lower. For now the sample point is fix @75%
    while (!IS_FDCAN_DATA_TSEG1(ntq / nominalPrescaler)) {
        nominalPrescaler ++;
        if (!IS_FDCAN_NOMINAL_PRESCALER(nominalPrescaler)) {
            error("Could not determine good nominalPrescaler. Bad clock value\n");
        }
    }
    ntq = ntq / nominalPrescaler;

    obj->CanHandle.Init.NominalPrescaler = nominalPrescaler;
    obj->CanHandle.Init.NominalTimeSeg1 = ntq * 0.75;      // Phase_segment_1
    obj->CanHandle.Init.NominalTimeSeg2 = ntq - 1 - obj->CanHandle.Init.NominalTimeSeg1;      // Phase_segment_2
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


int can_write(can_t *obj, CAN_Message msg)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};

    // Configure Tx buffer message
    TxHeader.Identifier = msg.id;
    if (msg.format == CANStandard) {
        TxHeader.IdType = FDCAN_STANDARD_ID;
    } else {
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    }

    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = msg.len << 16;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&obj->CanHandle, &TxHeader, msg.data) != HAL_OK)
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

int can_read(can_t *obj, CAN_Message *msg)
{
    if (HAL_FDCAN_GetRxFifoFillLevel(&obj->CanHandle, FDCAN_RX_FIFO0) == 0) {
        return 0; // No message arrived
    }

    FDCAN_RxHeaderTypeDef RxHeader = {0};
    if (HAL_FDCAN_GetRxMessage(&obj->CanHandle, FDCAN_RX_FIFO0, &RxHeader, msg->data) != HAL_OK) {
        error("HAL_FDCAN_GetRxMessage error\n"); // Should not occur as previous HAL_FDCAN_GetRxFifoFillLevel call reported some data
        return 0;
    }

    if (RxHeader.IdType == FDCAN_STANDARD_ID) {
        msg->format = CANStandard;
    } else {
        msg->format = CANExtended;
    }
    msg->id   = RxHeader.Identifier;
    msg->type = (RxHeader.RxFrameType == FDCAN_DATA_FRAME) ? CANData : CANRemote;
    msg->len  = RxHeader.DataLength >> 16; // see FDCAN_data_length_code value

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

void can_monitor(can_t *obj, int silent)
{
    CanMode mode = MODE_NORMAL;
    if (silent) {
        switch (obj->CanHandle.Init.Mode) {
            case FDCAN_MODE_INTERNAL_LOOPBACK:
                mode = MODE_TEST_SILENT;
                break;
            default:
                mode = MODE_SILENT;
                break;
        }
    } else {
        switch (obj->CanHandle.Init.Mode) {
            case FDCAN_MODE_INTERNAL_LOOPBACK:
            case FDCAN_MODE_EXTERNAL_LOOPBACK:
                mode = MODE_TEST_LOCAL;
                break;
            default:
                mode = MODE_NORMAL;
                break;
        }
    }

    can_mode(obj, mode);
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

#if 0
static void can_irq(CANName name, int id)
{
    FDCAN_HandleTypeDef CanHandle;
    CanHandle.Instance = (FDCAN_GlobalTypeDef *)name;

    if (__HAL_FDCAN_GET_IT_SOURCE(&CanHandle, FDCAN_IT_TX_COMPLETE)) {
        if (__HAL_FDCAN_GET_FLAG(&CanHandle, FDCAN_FLAG_TX_COMPLETE)) {
            __HAL_FDCAN_CLEAR_FLAG(&CanHandle, FDCAN_FLAG_TX_COMPLETE);
            irq_handler(can_irq_contexts[id], IRQ_TX);
        }
    }
#if (defined FDCAN_IT_RX_BUFFER_NEW_MESSAGE)
    if (__HAL_FDCAN_GET_IT_SOURCE(&CanHandle, FDCAN_IT_RX_BUFFER_NEW_MESSAGE)) {
        if (__HAL_FDCAN_GET_FLAG(&CanHandle, FDCAN_IT_RX_BUFFER_NEW_MESSAGE)) {
            __HAL_FDCAN_CLEAR_FLAG(&CanHandle, FDCAN_IT_RX_BUFFER_NEW_MESSAGE);
            irq_handler(can_irq_contexts[id], IRQ_RX);
        }
    }
#else
    if (__HAL_FDCAN_GET_IT_SOURCE(&CanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        if (__HAL_FDCAN_GET_FLAG(&CanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
            __HAL_FDCAN_CLEAR_FLAG(&CanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
            irq_handler(can_irq_contexts[id], IRQ_RX);
        }
    }
#endif
    if (__HAL_FDCAN_GET_IT_SOURCE(&CanHandle, FDCAN_IT_ERROR_WARNING)) {
        if (__HAL_FDCAN_GET_FLAG(&CanHandle, FDCAN_FLAG_ERROR_WARNING)) {
            __HAL_FDCAN_CLEAR_FLAG(&CanHandle, FDCAN_FLAG_ERROR_WARNING);
            irq_handler(can_irq_contexts[id], IRQ_ERROR);
        }
    }

    if (__HAL_FDCAN_GET_IT_SOURCE(&CanHandle, FDCAN_IT_ERROR_PASSIVE)) {
        if (__HAL_FDCAN_GET_FLAG(&CanHandle, FDCAN_FLAG_ERROR_PASSIVE)) {
            __HAL_FDCAN_CLEAR_FLAG(&CanHandle, FDCAN_FLAG_ERROR_PASSIVE);
            irq_handler(can_irq_contexts[id], IRQ_PASSIVE);
        }
    }

    if (__HAL_FDCAN_GET_IT_SOURCE(&CanHandle, FDCAN_IT_BUS_OFF)) {
        if (__HAL_FDCAN_GET_FLAG(&CanHandle, FDCAN_FLAG_BUS_OFF)) {
            __HAL_FDCAN_CLEAR_FLAG(&CanHandle, FDCAN_FLAG_BUS_OFF);
            irq_handler(can_irq_contexts[id], IRQ_BUS);
        }
    }
}
#endif

#if 0
void FDCAN1_IT0_IRQHandler(void)
{
    can_irq(CAN_1, 0);
}

void FDCAN1_IT1_IRQHandler(void)
{
    can_irq(CAN_1, 0);
}

#if defined(FDCAN2_BASE)
void FDCAN2_IT0_IRQHandler(void)
{
    can_irq(CAN_2, 1);
}

void FDCAN2_IT1_IRQHandler(void)
{
    can_irq(CAN_2, 1);
}
#endif //FDCAN2_BASE

#if defined(FDCAN3_BASE)
void FDCAN3_IT0_IRQHandler(void)
{
    can_irq(CAN_3, 2);
}

void FDCAN3_IT1_IRQHandler(void)
{
    can_irq(CAN_3, 2);
}
#endif //FDCAN3_BASE
#endif

// TODO Add other interrupts ?
void can_irq_set(can_t *obj, CanIrqType type, uint32_t enable)
{
    uint32_t interrupts = 0;

    switch (type) {
        case IRQ_TX:
            interrupts = FDCAN_IT_TX_COMPLETE;
            break;
        case IRQ_RX:
#if (defined FDCAN_IT_RX_BUFFER_NEW_MESSAGE)
            interrupts = FDCAN_IT_RX_BUFFER_NEW_MESSAGE;
#else
            interrupts = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
#endif
            break;
        case IRQ_ERROR:
            interrupts = FDCAN_IT_ERROR_WARNING;
            break;
        case IRQ_PASSIVE:
            interrupts = FDCAN_IT_ERROR_PASSIVE;
            break;
        case IRQ_BUS:
            interrupts = FDCAN_IT_BUS_OFF;
        default:
            return;
    }

    if (enable) {
        /* The TXBTIE register controls the TX complete interrupt in FDCAN 
         * and is only used in case of TX interrupts, Hence in case of enabling the 
         * TX interrupts the bufferIndexes of TXBTIE are to be set  */
#ifdef TARGET_STM32H7
        // TXBTIE for STM32H7 is 2 bytes long
        HAL_FDCAN_ActivateNotification(&obj->CanHandle, interrupts, 0xFFFF);
#else
        //TXBTIE for rest supported FDCAN Platforms(STM32G0x1, STM32G4 and STM32L5) is 3 bits.
        HAL_FDCAN_ActivateNotification(&obj->CanHandle, interrupts, 0x07);
#endif
    } else {
        HAL_FDCAN_DeactivateNotification(&obj->CanHandle, interrupts);
    }

#if 0
    NVIC_SetVector(FDCAN1_IT0_IRQn, (uint32_t)&FDCAN1_IT0_IRQHandler);
    NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    NVIC_SetVector(FDCAN1_IT1_IRQn, (uint32_t)&FDCAN1_IT1_IRQHandler);
    NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
#if defined(FDCAN2_BASE)
    NVIC_SetVector(FDCAN2_IT0_IRQn, (uint32_t)&FDCAN2_IT0_IRQHandler);
    NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    NVIC_SetVector(FDCAN2_IT1_IRQn, (uint32_t)&FDCAN2_IT1_IRQHandler);
    NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
#endif
#if defined(FDCAN3_BASE)
    NVIC_SetVector(FDCAN3_IT0_IRQn, (uint32_t)&FDCAN3_IT0_IRQHandler);
    NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    NVIC_SetVector(FDCAN3_IT1_IRQn, (uint32_t)&FDCAN3_IT1_IRQHandler);
    NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
#endif
#endif
}