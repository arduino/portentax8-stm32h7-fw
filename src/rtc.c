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

#include "rtc.h"
#include "main.h"
#include "system.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

/**************************************************************************************
 * GLOBAL VARIABLE
 **************************************************************************************/

RTC_HandleTypeDef hrtc;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void MX_RTC_Init(void) {

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }

}

/**
 * @brief RTC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (hrtc->Instance == RTC) {
    /* USER CODE BEGIN RTC_MspInit 0 */

    /* USER CODE END RTC_MspInit 0 */
    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    /* USER CODE BEGIN RTC_MspInit 1 */

    /* USER CODE END RTC_MspInit 1 */
  }
}

/**
 * @brief RTC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc) {
  if (hrtc->Instance == RTC) {
    /* USER CODE BEGIN RTC_MspDeInit 0 */

    /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    /* USER CODE BEGIN RTC_MspDeInit 1 */

    /* USER CODE END RTC_MspDeInit 1 */
  }
}

void rtc_handler(uint8_t opcode, uint8_t *data, uint16_t size) {
  if (opcode == SET_DATE) {
    rtc_set_date(data);
  }
  if (opcode == GET_DATE) {
    rtc_get_date(data);
  }
}

void rtc_init() {
  MX_RTC_Init();

  /*
  register_peripheral_callback(PERIPH_RTC, SET_DATE, &rtc_set_date);
  register_peripheral_callback(PERIPH_RTC, GET_DATE, &rtc_get_date);
  */

  register_peripheral_callback(PERIPH_RTC, &rtc_handler);

}

void rtc_set_date(uint8_t *data) {
  struct rtc_time *tm = (struct rtc_time*)data;
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  sTime.Hours = tm->tm_hour;
  sTime.Minutes = tm->tm_min;
  sTime.Seconds = tm->tm_sec;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();
  }
  sDate.WeekDay = tm->tm_wday;
  sDate.Month = (tm->tm_mon + RTC_MONTH_JANUARY); /* tm_mon is 0-11 while RTC_DateTypeDef expects 1-12 */
  sDate.Date = tm->tm_mday;
  sDate.Year = tm->tm_year;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();
  }
}

void rtc_get_date(uint8_t *data) {
  struct rtc_time *tm = (struct rtc_time*)data;
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  struct rtc_time now;

  if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();
  }

  now.tm_hour = sTime.Hours;
  now.tm_min = sTime.Minutes;
  now.tm_sec = sTime.Seconds;
  now.tm_wday = sDate.WeekDay;
  now.tm_mon = (sDate.Month - RTC_MONTH_JANUARY); /* tm_mon is 0-11 while RTC_DateTypeDef expects 1-12 */
  now.tm_mday = sDate.Date;
  now.tm_year = sDate.Year;

  enqueue_packet(PERIPH_RTC, GET_DATE, sizeof(now), &now);

/*
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();
  }

  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) !=
      HAL_OK) {
    Error_Handler();
  }
*/

}