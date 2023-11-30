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

#ifndef PORTENTAX8_STM32H7_FW_CAN_MSG_RINGBUFFER_H
#define PORTENTAX8_STM32H7_FW_CAN_MSG_RINGBUFFER_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include "can.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

union can_frame
{
  struct __attribute__((packed))
  {
    uint32_t id;                                // 29 bit identifier
    uint8_t  len;                               // Length of data field in bytes
    uint8_t  data[X8H7_CAN_FRAME_MAX_DATA_LEN]; // Data field
  } field;
  uint8_t buf[X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN];
};

typedef struct CanMsgRingbuffer CanMsgRingbuffer;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

struct CanMsgRingbuffer * can_msg_ringbuffer_create(void);
void can_msg_ringbuffer_destroy(struct CanMsgRingbuffer* buffer);

int can_msg_ringbuffer_is_full(const CanMsgRingbuffer* buffer);
void can_msg_ringbuffer_enqueue(struct CanMsgRingbuffer * buffer, union can_frame const * msg);

int can_msg_ringbuffer_is_empty(struct CanMsgRingbuffer const * buffer);
void can_msg_ringbuffer_dequeue(struct CanMsgRingbuffer * buffer, union can_frame * msg);

size_t can_msg_ringbuffer_available(struct CanMsgRingbuffer const * buffer);

#endif /* PORTENTAX8_STM32H7_FW_CAN_MSG_RINGBUFFER_H */
