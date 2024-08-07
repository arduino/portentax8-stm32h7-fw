
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2016 ARM Limited
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

#ifndef MBED_CAN_API_H
#define MBED_CAN_API_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdbool.h>
#include <inttypes.h>

#include "stm32h7xx_hal.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define X8H7_CAN_HEADER_SIZE        5
#define X8H7_CAN_FRAME_MAX_DATA_LEN	8

/* Special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

/* Valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef enum {
    CAN_1 = (int)FDCAN1_BASE,
    CAN_2 = (int)FDCAN2_BASE
} CANName;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void          can_init(FDCAN_HandleTypeDef * handle, CANName peripheral, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width);
void          can_deinit(FDCAN_HandleTypeDef * handle);
int           can_set_bittiming(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width);

uint32_t      can_tx_fifo_available(FDCAN_HandleTypeDef * handle);
uint32_t      can_rx_fifo_available(FDCAN_HandleTypeDef * handle, uint32_t const rx_fifo);
int           can_write(FDCAN_HandleTypeDef * handle, uint32_t const id, uint8_t const len, uint8_t const * data);
int           can_read(FDCAN_HandleTypeDef * handle, uint32_t * id, uint8_t * len, uint8_t * data);
int           can_filter(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask, bool const is_extended_id);
unsigned char can_rderror(FDCAN_HandleTypeDef * handle);
unsigned char can_tderror(FDCAN_HandleTypeDef * handle);

#endif    // MBED_CAN_API_H
