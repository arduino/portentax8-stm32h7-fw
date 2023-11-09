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
 * GLOBAL VARIABLES
 **************************************************************************************/

extern FDCAN_HandleTypeDef fdcan_1;
extern FDCAN_HandleTypeDef fdcan_2;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

static int on_CAN_CONFIGURE_Request(FDCAN_HandleTypeDef * handle, uint32_t const can_bitrate);

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int fdcan1_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CONFIGURE)
  {
    uint32_t const can_bitrate = *((uint32_t *)data);
    on_CAN_CONFIGURE_Request(&fdcan_1, can_bitrate);
    dbg_printf("fdcan1_handler: initializing fdcan1 with frequency %ld\n", can_bitrate);
    return 0;
  }
  else if (opcode == CAN_FILTER)
  {
    union x8h7_can_filter_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));

    if (!can_filter(&fdcan_1,
                    x8h7_msg.field.idx,
                    x8h7_msg.field.id,
                    x8h7_msg.field.mask,
                    x8h7_msg.field.id & CAN_EFF_FLAG))
    {
      dbg_printf("fdcan1_handler: can_filter failed for idx: %ld, id: %lX, mask: %lX\n", x8h7_msg.field.idx, x8h7_msg.field.id, x8h7_msg.field.mask);
    }
    return 0;
  }
  else if (opcode == CAN_TX_FRAME)
  {
    union x8h7_can_frame_message msg;
    memcpy(&msg, data, size);

    dbg_printf("fdcan1_handler: sending CAN message to %lx, size %d, content[0]=0x%02X\n", msg.field.id, msg.field.len, msg.field.data[0]);
    return can_write(&fdcan_1, &msg);
  }
  else
  {
    dbg_printf("fdcan1_handler: error invalid opcode (:%d)\n", opcode);
    return 0;
  }
}

int fdcan2_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  if (opcode == CONFIGURE)
  {
    uint32_t const can_bitrate = *((uint32_t *)data);
    on_CAN_CONFIGURE_Request(&fdcan_2, can_bitrate);
    dbg_printf("fdcan2_handler: initializing fdcan2 with frequency %ld\n", can_bitrate);
    return 0;
  }
  else if (opcode == CAN_FILTER)
  {
    union x8h7_can_filter_message x8h7_msg;
    memcpy(x8h7_msg.buf, data, sizeof(x8h7_msg.buf));

    if (!can_filter(&fdcan_2,
                    x8h7_msg.field.idx,
                    x8h7_msg.field.id,
                    x8h7_msg.field.mask,
                    x8h7_msg.field.id & CAN_EFF_FLAG))
    {
      dbg_printf("fdcan2_handler: can_filter failed for idx: %ld, id: %lX, mask: %lX\n", x8h7_msg.field.idx, x8h7_msg.field.id, x8h7_msg.field.mask);
    }
    return 0;
  }
  else if (opcode == CAN_TX_FRAME)
  {
    union x8h7_can_frame_message msg;
    memcpy(&msg, data, size);

    dbg_printf("fdcan2_handler: sending CAN message to %lx, size %d, content[0]=0x%02X\n", msg.field.id, msg.field.len, msg.field.data[0]);
    return can_write(&fdcan_2, &msg);
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

int on_CAN_CONFIGURE_Request(FDCAN_HandleTypeDef * handle, uint32_t const can_bitrate)
{
  CanNominalBitTimingResult can_bit_timing = {0};

  if (!calc_can_nominal_bit_timing(can_bitrate,
                                   HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN),
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
