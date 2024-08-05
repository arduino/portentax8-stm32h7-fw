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

#include "can_handler.h"

#include <string.h>

#include "stm32h7xx_hal.h"

#include "can.h"
#include "debug.h"
#include "system.h"
#include "opcodes.h"
#include "peripherals.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define X8H7_CAN_STS_FLG_RX_OVR  0x01  // Receive Buffer Overflow
#define X8H7_CAN_STS_FLG_TX_BO   0x02  // Bus-Off
#define X8H7_CAN_STS_FLG_TX_EP   0x04  // Transmit Error-Passive
#define X8H7_CAN_STS_FLG_RX_EP   0x08  // Receive Error-Passive
#define X8H7_CAN_STS_FLG_TX_WAR  0x10  // Transmit Error Warning
#define X8H7_CAN_STS_FLG_RX_WAR  0x20  // Receive Error Warning
#define X8H7_CAN_STS_FLG_EWARN   0x40  // Error Warning
#define X8H7_CAN_STS_FLG_TX_OVR  0x80  // Transmit Buffer Overflow

#define X8H7_CAN_STS_INT_TX_COMPLETE       0x01
#define X8H7_CAN_STS_INT_RX                0x02
#define X8H7_CAN_STS_INT_ERR               0x04
#define X8H7_CAN_STS_INT_TX_ABORT_COMPLETE 0x08
#define X8H7_CAN_STS_INT_TX_FIFO_EMPTY     0x10

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

union x8h7_can_init_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
  } field;
  uint8_t buf[sizeof(uint32_t) /* can_bitrate_Hz */ + sizeof(uint32_t) /* time_segment_1 */ + sizeof(uint32_t) /* time_segment_2 */ + sizeof(uint32_t) /* sync_jump_width */];
};

union x8h7_can_bittiming_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
  } field;
  uint8_t buf[sizeof(uint32_t) /* can_bitrate_Hz */ + sizeof(uint32_t) /* time_segment_1 */ + sizeof(uint32_t) /* time_segment_2 */ + sizeof(uint32_t) /* sync_jump_width */];
};

union x8h7_can_filter_message
{
  struct __attribute__((packed))
  {
    uint32_t idx;
    uint32_t id;
    uint32_t mask;
  } field;
  uint8_t buf[sizeof(uint32_t) /* idx */ + sizeof(uint32_t) /* id */ + sizeof(uint32_t) /* mask */];
};

union x8h7_can_frame_message
{
  struct __attribute__((packed))
  {
    uint32_t id;                           // 29 bit identifier
    uint8_t  len;                          // Length of data field in bytes
    uint8_t  data[X8H7_CAN_FRAME_MAX_DATA_LEN]; // Data field
  } field;
  uint8_t buf[X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN];
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

extern FDCAN_HandleTypeDef fdcan_1;
extern FDCAN_HandleTypeDef fdcan_2;

static bool is_can1_init = false;
static bool is_can2_init = false;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

static int fdcan_handler(FDCAN_HandleTypeDef * handle, uint8_t const opcode, uint8_t const * data, uint16_t const size);
static int on_CAN_INIT_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width);
static int on_CAN_DEINIT_Request(FDCAN_HandleTypeDef * handle);
static int on_CAN_SET_BITTIMING_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width);
static int on_CAN_FILTER_Request(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask);
static int on_CAN_TX_FRAME_Request(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg);

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int can_handle_data()
{
  int bytes_enqueued = 0;
  uint32_t can_id = 0;
  uint8_t can_len = 0;
  uint8_t can_data[X8H7_CAN_FRAME_MAX_DATA_LEN] = {0};

  /* Note: the last read package is lost in this implementation. We need to fix this by
   * implementing some peek method or by buffering messages in a ringbuffer.
   */
  int rc_enq;

  if (is_can1_init)
  {
    for (rc_enq = 0; can_read(&fdcan_1, &can_id, &can_len, can_data); bytes_enqueued += rc_enq)
    {
      union x8h7_can_frame_message x8h7_msg;

      x8h7_msg.field.id = can_id;
      x8h7_msg.field.len = can_len;
      memcpy(x8h7_msg.field.data, can_data, x8h7_msg.field.len);

      rc_enq = enqueue_packet(PERIPH_FDCAN1, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + x8h7_msg.field.len, x8h7_msg.buf);
      if (!rc_enq) break;
    }
  }

  if (is_can2_init)
  {
    for (rc_enq = 0; can_read(&fdcan_2, &can_id, &can_len, can_data); bytes_enqueued += rc_enq)
    {
      union x8h7_can_frame_message x8h7_msg;

      x8h7_msg.field.id = can_id;
      x8h7_msg.field.len = can_len;
      memcpy(x8h7_msg.field.data, can_data, x8h7_msg.field.len);

      rc_enq = enqueue_packet(PERIPH_FDCAN2, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + x8h7_msg.field.len, x8h7_msg.buf);
      if (!rc_enq) break;
    }
  }

  if (!rc_enq) {
    // Report dropped rx packets next time there's a slot
    // uint8_t x8_msg[2] = {X8H7_CAN_STS_FLG_RX_OVR, can_tx_fifo_available(handle)};
    // enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
  }

  return bytes_enqueued;
}

int fdcan1_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  dbg_printf("fdcan1_handler\n");
  if (!is_can1_init && opcode != CAN_INIT) return 0;
  else return fdcan_handler(&fdcan_1, opcode, data, size);
}

int fdcan2_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  dbg_printf("fdcan2_handler\n");
  if (!is_can2_init && opcode != CAN_INIT) return 0;
  else return fdcan_handler(&fdcan_2, opcode, data, size);
}

int fdcan_handler(FDCAN_HandleTypeDef * handle, uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CAN_INIT)
  {
    dbg_printf("fdcan_handler: CAN_INIT\n");
    union x8h7_can_init_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));

    return on_CAN_INIT_Request(handle,
                               x8h7_msg.field.baud_rate_prescaler,
                               x8h7_msg.field.time_segment_1,
                               x8h7_msg.field.time_segment_2,
                               x8h7_msg.field.sync_jump_width);
  }
  else if (opcode == CAN_DEINIT)
  {
    dbg_printf("fdcan_handler: CAN_DEINIT\n");
    return on_CAN_DEINIT_Request(handle);
  }
  else if (opcode == CAN_SET_BITTIMING)
  {
    dbg_printf("fdcan_handler: CAN_SET_BITTIMING\n");
    union x8h7_can_bittiming_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));

    return on_CAN_SET_BITTIMING_Request(handle,
                                        x8h7_msg.field.baud_rate_prescaler,
                                        x8h7_msg.field.time_segment_1,
                                        x8h7_msg.field.time_segment_2,
                                        x8h7_msg.field.sync_jump_width);
  }
  else if (opcode == CAN_FILTER)
  {
    union x8h7_can_filter_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));
    dbg_printf("fdcan_handler: CAN_FILTER\n");
    return on_CAN_FILTER_Request(handle,
                                 x8h7_msg.field.idx,
                                 x8h7_msg.field.id,
                                 x8h7_msg.field.mask);
  }
  else if (opcode == CAN_TX_FRAME)
  {
    union x8h7_can_frame_message msg;
    memcpy(&msg, data, size);
    dbg_printf("fdcan_handler: sending CAN message to %lx, size %d, content[0]=0x%02X\n", msg.field.id, msg.field.len, msg.field.data[0]);
    return on_CAN_TX_FRAME_Request(handle, &msg);
  }
  else
  {
    dbg_printf("fdcan_handler: error invalid opcode (:%d)\n", opcode);
    return 0;
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int on_CAN_INIT_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width)
{
  can_init(handle,
           (handle == &fdcan_1) ? CAN_1 : CAN_2,
           baud_rate_prescaler,
           time_segment_1,
           time_segment_2,
           sync_jump_width);

  if      (handle == &fdcan_1) is_can1_init = true;
  else if (handle == &fdcan_2) is_can2_init = true;

  return 0;
}

int on_CAN_DEINIT_Request(FDCAN_HandleTypeDef * handle)
{
  can_deinit(handle);

  if      (handle == &fdcan_1) is_can1_init = false;
  else if (handle == &fdcan_2) is_can2_init = false;

  return 0;
}

int on_CAN_SET_BITTIMING_Request(FDCAN_HandleTypeDef * handle, uint32_t const baud_rate_prescaler, uint32_t const time_segment_1, uint32_t const time_segment_2, uint32_t const sync_jump_width)
{
  return can_set_bittiming(handle,
                           baud_rate_prescaler,
                           time_segment_1,
                           time_segment_2,
                           sync_jump_width);
}

int on_CAN_FILTER_Request(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask)
{
  if (!can_filter(handle, filter_index, id, mask, id & CAN_EFF_FLAG))
    dbg_printf("fdcan2_handler: can_filter failed for idx: %ld, id: %lX, mask: %lX\n", filter_index, id, mask);
  return 0;
}

int on_CAN_TX_FRAME_Request(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg)
{
  if (!can_tx_fifo_available(handle))
  {
    uint8_t x8_msg[2] = {X8H7_CAN_STS_INT_ERR, X8H7_CAN_STS_FLG_TX_OVR};
    return enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
  }

  int const rc = can_write(handle, msg->field.id, msg->field.len, msg->field.data);
  if (rc < 0)
  {
    uint8_t x8_msg[2] = {X8H7_CAN_STS_INT_ERR, X8H7_CAN_STS_FLG_TX_EP};
    return enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
  }
  return 0;
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef * handle, uint32_t BufferIndexes)
{
  uint8_t x8_msg[2] = {X8H7_CAN_STS_INT_TX_COMPLETE, BufferIndexes};
  enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef * handle, uint32_t BufferIndexes)
{
  uint8_t x8_msg[2] = {X8H7_CAN_STS_INT_TX_ABORT_COMPLETE, BufferIndexes};
  enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef * handle)
{
  uint8_t x8_msg[2] = {X8H7_CAN_STS_INT_TX_FIFO_EMPTY, can_tx_fifo_available(handle)};
  enqueue_packet(handle == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_STATUS, sizeof(x8_msg), x8_msg);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint32_t can_id = 0;
  uint8_t can_len = 0;
  uint8_t can_data[X8H7_CAN_FRAME_MAX_DATA_LEN] = {0};

  if(((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) || (RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK)) != RESET)
  {
    union x8h7_can_frame_message x8h7_msg;

    while ((get_available_enqueue() >= sizeof(x8h7_msg)) && can_read(hfdcan, &can_id, &can_len, can_data))
    {
      x8h7_msg.field.id = can_id;
      x8h7_msg.field.len = can_len;
      memcpy(x8h7_msg.field.data, can_data, x8h7_msg.field.len);

      int ret = enqueue_packet(hfdcan == &fdcan_1 ? PERIPH_FDCAN1 : PERIPH_FDCAN2, CAN_RX_FRAME, X8H7_CAN_HEADER_SIZE + x8h7_msg.field.len, x8h7_msg.buf);
      if (ret == 0) break;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_WATERMARK, 0) != HAL_OK)
    {
      /* Notification Error */
    }
  }
}
