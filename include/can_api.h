
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

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <inttypes.h>

#include "can_util.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define X8H7_CAN_HEADER_SIZE        5
#define X8H7_CAN_FRAME_MAX_DATA_LEN	8

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef enum
{
    CANStandard = 0,
    CANExtended = 1,
    CANAny = 2
} CANFormat;

typedef enum
{
  CANData   = 0,
  CANRemote = 1
} CANType;

typedef struct __attribute__((packed, aligned(4)))
{
    unsigned int   id;                 // 29 bit identifier
    unsigned char  len;                // Length of data field in bytes
    unsigned char  data[8];            // Data field
    CANFormat      format;             // Format ::CANFormat
    CANType        type;               // Type ::CANType
} CAN_Message;

union x8h7_can_message
{
  struct __attribute__((packed))
  {
    uint32_t id;                           // 29 bit identifier
    uint8_t  len;                          // Length of data field in bytes
    uint8_t  data[X8H7_CAN_FRAME_MAX_DATA_LEN]; // Data field
  } field;
  uint8_t buf[X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN];
};

typedef enum {
    MODE_RESET,
    MODE_NORMAL,
    MODE_SILENT,
    MODE_TEST_LOCAL,
    MODE_TEST_GLOBAL,
    MODE_TEST_SILENT
} CanMode;

typedef enum {
    CAN_1 = (int)FDCAN1_BASE,
    CAN_2 = (int)FDCAN2_BASE
} CANName;

typedef struct
{
    FDCAN_HandleTypeDef CanHandle;
    int index;
    int hz;
} can_t;

void          canInit();
void          can_handle_data();

void          can_init(can_t *obj, CANName peripheral, CanNominalBitTimingResult const can_bit_timing);
int           can_frequency(can_t *obj, uint32_t const can_bitrate);

int           can_write(can_t *obj, CAN_Message);
int           can_read(can_t *obj, union x8h7_can_message *msg);
int           can_mode(can_t *obj, CanMode mode);
int           can_filter(can_t *obj, uint32_t id, uint32_t mask, CANFormat format, int32_t handle);
void          can_reset(can_t *obj);
unsigned char can_rderror(can_t *obj);
unsigned char can_tderror(can_t *obj);

#ifdef __cplusplus
}
#endif

#endif    // MBED_CAN_API_H

/** @}*/
