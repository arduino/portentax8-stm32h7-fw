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
#include "opcodes.h"
#include "error_handler.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

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

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint32_t const CAN_PERIPHERAL_CLOCK_Hz = 100*1000*1000UL;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

extern FDCAN_HandleTypeDef fdcan_1;
extern FDCAN_HandleTypeDef fdcan_2;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

static int on_CAN_INIT_Request(FDCAN_HandleTypeDef * handle, uint32_t const can_bitrate);
static int on_CAN_DEINIT_Request(FDCAN_HandleTypeDef * handle);
static int on_CAN_FILTER_Request(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask);
static int on_CAN_TX_FRAME_Request(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg);

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int fdcan1_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CAN_INIT)
  {
    uint32_t const can_bitrate = *((uint32_t *)data);
    dbg_printf("fdcan1_handler: initializing with frequency %ld\n", can_bitrate);
    return on_CAN_INIT_Request(&fdcan_1, can_bitrate);
  }
  else if (opcode == CAN_DEINIT)
  {
    dbg_printf("fdcan1_handler: CAN_DEINIT\n");
    return on_CAN_DEINIT_Request(&fdcan_1);
  }
  else if (opcode == CAN_FILTER)
  {
    union x8h7_can_filter_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));
    dbg_printf("fdcan1_handler: CAN_FILTER\n");
    return on_CAN_FILTER_Request(&fdcan_1,
                                 x8h7_msg.field.idx,
                                 x8h7_msg.field.id,
                                 x8h7_msg.field.mask);
  }
  else if (opcode == CAN_TX_FRAME)
  {
    union x8h7_can_frame_message msg;
    memcpy(&msg, data, size);
    dbg_printf("fdcan1_handler: sending CAN message to %lx, size %d, content[0]=0x%02X\n", msg.field.id, msg.field.len, msg.field.data[0]);
    return on_CAN_TX_FRAME_Request(&fdcan_1, &msg);
  }
  else
  {
    dbg_printf("fdcan1_handler: error invalid opcode (:%d)\n", opcode);
    return 0;
  }
}

int fdcan2_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CAN_INIT)
  {
    uint32_t const can_bitrate = *((uint32_t *)data);
    dbg_printf("fdcan2_handler: initializing with frequency %ld\n", can_bitrate);
    return on_CAN_INIT_Request(&fdcan_2, can_bitrate);
  }
  else if (opcode == CAN_DEINIT)
  {
    dbg_printf("fdcan2_handler: CAN_DEINIT\n");
    return on_CAN_DEINIT_Request(&fdcan_2);
  }
  else if (opcode == CAN_FILTER)
  {
    union x8h7_can_filter_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));
    dbg_printf("fdcan2_handler: CAN_FILTER\n");
    return on_CAN_FILTER_Request(&fdcan_2,
                                 x8h7_msg.field.idx,
                                 x8h7_msg.field.id,
                                 x8h7_msg.field.mask);
  }
  else if (opcode == CAN_TX_FRAME)
  {
    union x8h7_can_frame_message msg;
    memcpy(&msg, data, size);
    dbg_printf("fdcan2_handler: sending CAN message to %lx, size %d, content[0]=0x%02X\n", msg.field.id, msg.field.len, msg.field.data[0]);
    return on_CAN_TX_FRAME_Request(&fdcan_2, &msg);
  }
  else
  {
    dbg_printf("fdcan2_handler: error invalid opcode (:%d)\n", opcode);
    return 0;
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int on_CAN_INIT_Request(FDCAN_HandleTypeDef * handle, uint32_t const can_bitrate)
{
  CanNominalBitTimingResult can_bit_timing = {0};

  if (!calc_can_nominal_bit_timing(can_bitrate,
                                   CAN_PERIPHERAL_CLOCK_Hz,
                                   TQ_MAX,
                                   TQ_MIN,
                                   TSEG1_MIN,
                                   TSEG1_MAX,
                                   TSEG2_MIN,
                                   TSEG2_MAX,
                                   &can_bit_timing))
  {
    Error_Handler("Could not calculate valid CAN bit timing\n");
  }

  can_init_device(handle,
                  (handle == &fdcan_1) ? CAN_1 : CAN_2,
                  can_bit_timing);

  return 0;
}

int on_CAN_DEINIT_Request(FDCAN_HandleTypeDef * handle)
{
  can_deinit_device(handle);
  return 0;
}

int on_CAN_FILTER_Request(FDCAN_HandleTypeDef * handle, uint32_t const filter_index, uint32_t const id, uint32_t const mask)
{
  if (!can_filter(handle, filter_index, id, mask, id & CAN_EFF_FLAG))
    dbg_printf("fdcan2_handler: can_filter failed for idx: %ld, id: %lX, mask: %lX\n", filter_index, id, mask);
  return 0;
}

int on_CAN_TX_FRAME_Request(FDCAN_HandleTypeDef * handle, union x8h7_can_frame_message const * msg)
{
  return can_write(handle, msg);
}
