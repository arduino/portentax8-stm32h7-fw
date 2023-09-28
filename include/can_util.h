/*
 * Copyright (c) 2022 by Alexander Entinger <a.entinger@arduino.cc>
 * CAN library for Arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef PORTENTA_X8_FIRMWARE_CAN_UTIL_H_
#define PORTENTA_X8_FIRMWARE_CAN_UTIL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef struct
{
  uint32_t baud_rate_prescaler;
  uint32_t time_segment_1;
  uint32_t time_segment_2;
} CanNominalBitTimingResult;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool calc_can_nominal_bit_timing(
  uint32_t const can_bitrate,
  uint32_t const can_clock_Hz,
  uint32_t const tq_max,
  uint32_t const tq_min,
  uint32_t const tseg1_min,
  uint32_t const tseg1_max,
  uint32_t const tseg2_min,
  uint32_t const tseg2_max,
  CanNominalBitTimingResult * can_bit_timing);

#endif /* PORTENTA_X8_FIRMWARE_CAN_UTIL_H_ */
