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

#include "can_msg_ringbuffer.h"

#include <stdlib.h>
#include <string.h>

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define CanMsgRingbuffer_SIZE 64

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

struct CanMsgRingbuffer
{
  union can_frame buf[CanMsgRingbuffer_SIZE];
  size_t head;
  size_t tail;
  size_t num_elems;
};

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

struct CanMsgRingbuffer * can_msg_ringbuffer_create(void)
{
  struct CanMsgRingbuffer* buffer = (struct CanMsgRingbuffer *)malloc(sizeof(CanMsgRingbuffer));
  if (buffer) {
    buffer->head = 0;
    buffer->tail = 0;
    buffer->num_elems = 0;
  }
  return buffer;
}

void can_msg_ringbuffer_destroy(struct CanMsgRingbuffer * buffer)
{
  free(buffer);
}

int can_msg_ringbuffer_is_full(struct CanMsgRingbuffer const * buffer)
{
  return (buffer->num_elems == CanMsgRingbuffer_SIZE);
}

void can_msg_ringbuffer_enqueue(struct CanMsgRingbuffer* buffer, union can_frame const * msg)
{
  if (!can_msg_ringbuffer_is_full(buffer))
  {
    memcpy(buffer->buf[buffer->head].buf, msg->buf, sizeof(msg->buf));
    buffer->head = (buffer->head + 1) % CanMsgRingbuffer_SIZE;
    buffer->num_elems++;
  }
}

int can_msg_ringbuffer_is_empty(struct CanMsgRingbuffer const * buffer)
{
  return (buffer->num_elems == 0);
}

void can_msg_ringbuffer_dequeue(struct CanMsgRingbuffer * buffer, union can_frame * msg)
{
  if (!can_msg_ringbuffer_is_empty(buffer))
  {
    memcpy(msg->buf, buffer->buf[buffer->tail].buf, sizeof(msg->buf));
    buffer->tail = (buffer->tail + 1) % CanMsgRingbuffer_SIZE;
    buffer->num_elems--;
  }
}

size_t can_msg_ringbuffer_available(struct CanMsgRingbuffer const * buffer)
{
  return buffer->num_elems;
}
