
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

#include <inttypes.h>

/**
 *
 * \enum    CANFormat
 *
 * \brief   Values that represent CAN Format
**/
enum CANFormat {
    CANStandard = 0,
    CANExtended = 1,
    CANAny = 2
};
typedef enum CANFormat CANFormat;

/**
 *
 * \enum    CANType
 *
 * \brief   Values that represent CAN Type
**/
enum CANType {
    CANData   = 0,
    CANRemote = 1
};
typedef enum CANType CANType;

/**
 *
 * \struct  CAN_Message
 *
 * \brief   Holder for single CAN message.
 *
**/
__attribute__((packed, aligned(4))) struct CAN_Message {
    unsigned int   id;                 // 29 bit identifier
    unsigned char  len;                // Length of data field in bytes
    unsigned char  data[8];            // Data field
    CANFormat      format;             // Format ::CANFormat
    CANType        type;               // Type ::CANType
};
typedef struct CAN_Message CAN_Message;

typedef enum {
    IRQ_RX,
    IRQ_TX,
    IRQ_ERROR,
    IRQ_OVERRUN,
    IRQ_WAKEUP,
    IRQ_PASSIVE,
    IRQ_ARB,
    IRQ_BUS,
    IRQ_READY
} CanIrqType;


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

typedef void (*can_irq_handler)(uint32_t id, CanIrqType type);

struct can_s {
    FDCAN_HandleTypeDef CanHandle;
    int index;
    int hz;
};

typedef struct can_s can_t;

void          canInit();
int           canFilter(uint8_t peripheral, uint32_t id, uint32_t mask, CANFormat format, int32_t handle);
void          can_handle_data();

void          can_init(can_t *obj);
void          can_init_direct(can_t *obj);
void          can_init_freq(can_t *obj, int hz);
void          can_init_freq_direct(can_t *obj, CANName peripheral, int hz);
void          can_free(can_t *obj);
int           can_frequency(can_t *obj, int hz);

void          can_irq_init(can_t *obj, can_irq_handler handler, uintptr_t id);
void          can_irq_free(can_t *obj);
void          can_irq_set(can_t *obj, CanIrqType irq, uint32_t enable);


int           can_write(can_t *obj, CAN_Message);
int           can_read(can_t *obj, CAN_Message *msg);
int           can_mode(can_t *obj, CanMode mode);
int           can_filter(can_t *obj, uint32_t id, uint32_t mask, CANFormat format, int32_t handle);
void          can_reset(can_t *obj);
unsigned char can_rderror(can_t *obj);
unsigned char can_tderror(can_t *obj);
void          can_monitor(can_t *obj, int silent);

#ifdef __cplusplus
}
#endif

#endif    // MBED_CAN_API_H

/** @}*/
