#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "peripherals.h"
#include "ringbuffer.h"
#include "can_api.h"
#include "rpc.h"
#include "stm32h7xx_ll_rcc.h"
#include "adc.h"

//#define PORTENTA_DEBUG_WIRED

HRTIM_HandleTypeDef hhrtim;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart2;

IWDG_HandleTypeDef watchdog;

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_HRTIM_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);

#undef DEBUG

#define SPI_DMA_BUFFER_SIZE 	2048

__attribute__((packed, aligned(4))) struct subpacket {
  uint8_t peripheral;
  uint8_t opcode;
  uint16_t size;
  uint8_t raw_data;
};

__attribute__((packed, aligned(4))) struct complete_packet {
  uint16_t size;
  uint16_t checksum;
  struct subpacket data;
  // ... other subpackets will follow
};

__attribute__((section("dma"), aligned(2048))) volatile uint8_t TX_Buffer[SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer[SPI_DMA_BUFFER_SIZE];

__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer_userspace[SPI_DMA_BUFFER_SIZE];

enum { TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_ERROR };

__IO uint32_t transferState = TRANSFER_WAIT;
volatile bool get_data_amount = true;
volatile uint16_t data_amount = 0;

#define max(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _a : _b;                                                         \
  })

struct __attribute__((packed, aligned(4))) pwmPacket {
	uint8_t enable: 1;
	uint8_t polarity: 1;
	uint32_t duty: 30;
	uint32_t period: 32;
};

enum UARTParity {
  PARITY_EVEN = 0,
  PARITY_ODD,
  PARITY_NONE,
};

struct __attribute__((packed, aligned(4))) uartPacket {
  uint8_t bits: 4;
  uint8_t stop_bits: 2;
  uint8_t parity: 2;
  uint8_t flow_control: 1;
  uint32_t baud: 23;
};

volatile bool trigger_irq = false;

struct rtc_time {
  uint8_t tm_sec;
  uint8_t tm_min;
  uint8_t tm_hour;
  uint8_t tm_mday;
  uint8_t tm_mon;
  uint8_t tm_year;
  uint8_t tm_wday;
};

void doRTCStuff(uint8_t opcode, struct rtc_time *tm) {
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  if (opcode == SET_DATE) {
    sTime.Hours = tm->tm_hour;
    sTime.Minutes = tm->tm_min;
    sTime.Seconds = tm->tm_sec;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
      Error_Handler();
    }
    sDate.WeekDay = tm->tm_wday;
    sDate.Month = tm->tm_mon;
    sDate.Date = tm->tm_mday;
    sDate.Year = tm->tm_year;
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
      Error_Handler();
    }
  }

  if (opcode == GET_DATE) {

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
    now.tm_mon = sDate.Month;
    now.tm_mday = sDate.Date;
    now.tm_year = sDate.Year;

    enqueue_packet(PERIPH_RTC, opcode, sizeof(now), &now);
  }

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

void writeVersion() {
  const char* version = "v0.1";
  enqueue_packet(PERIPH_H7, FW_VERSION, strlen(version), (void*)version);
}

void configurePwm(uint8_t channel, bool enable, bool polarity, uint32_t duty_ns, uint32_t period_ns) {
	dbg_printf("PWM channel %d %s with polarity %s, duty %dns, period %dns\n", channel, enable ? "enabled" : "disabled",
			polarity? "high": "low", duty_ns, period_ns);

  HRTIM_SimplePWMChannelCfgTypeDef sConfig_Channel = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};

  pTimeBaseCfg.Period = period_ns / 5;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;

  sConfig_Channel.Polarity = polarity ? HRTIM_OUTPUTPOLARITY_HIGH : HRTIM_OUTPUTPOLARITY_LOW;
  sConfig_Channel.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  sConfig_Channel.Pulse = duty_ns / 5;

  HAL_HRTIM_TimeBaseConfig(&hhrtim, PWM_pinmap[channel].index, &pTimeBaseCfg);
  HAL_HRTIM_SimplePWMChannelConfig(&hhrtim, PWM_pinmap[channel].index, PWM_pinmap[channel].channel, &sConfig_Channel);
  HAL_HRTIM_SoftwareUpdate(&hhrtim,HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_C
      | HRTIM_TIMERUPDATE_D | HRTIM_TIMERUPDATE_E);

  if (enable) {
    HAL_HRTIM_SimplePWMStart(&hhrtim, PWM_pinmap[channel].index, PWM_pinmap[channel].channel);
  } else {
    HAL_HRTIM_SimplePWMStop(&hhrtim, PWM_pinmap[channel].index, PWM_pinmap[channel].channel);
  }

  // If capture is needed, use code from https://github.com/kongr45gpen/stm32h7-freqcounter/blob/master/Src/main.c
}

void UART2_enable_rx_irq();

void configureUart(uint32_t baud, uint8_t bits, uint8_t parity, uint8_t stop_bits, bool flow_control) {

  //HAL_UART_DeInit(&huart2);

  uint32_t WordLength;
  uint32_t Parity;
  uint32_t StopBits;
  uint32_t HwFlowCtl = UART_HWCONTROL_NONE;

  switch (bits) {
    case 7:
      WordLength = UART_WORDLENGTH_7B;
      break;
    case 8:
      WordLength = UART_WORDLENGTH_8B;
      break;
    case 9:
      WordLength = UART_WORDLENGTH_9B;
      break;
  }

  char parity_str;

  switch (parity) {
    case PARITY_EVEN:
      Parity = UART_PARITY_EVEN;
      parity_str = 'E';
      break;
    case PARITY_ODD:
      Parity = UART_PARITY_ODD;
      parity_str = 'O';
      break;
    case PARITY_NONE:
      Parity = UART_PARITY_NONE;
      parity_str = 'N';
      break;
  }

  switch (stop_bits) {
    case 0:
      StopBits = UART_STOPBITS_0_5;
      break;
    case 1:
      StopBits = UART_STOPBITS_1;
      break;
    case 2:
      StopBits = UART_STOPBITS_1;
      break;
  }

  if (flow_control) {
    HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  }

  if (huart2.Init.BaudRate == baud &&
      huart2.Init.WordLength == WordLength &&
      huart2.Init.StopBits == StopBits &&
      huart2.Init.Parity == Parity &&
      huart2.Init.HwFlowCtl == HwFlowCtl) {
      return;
  }

  huart2.Init.BaudRate = baud;
  huart2.Init.WordLength = WordLength;
  huart2.Init.StopBits = StopBits;
  huart2.Init.Parity = Parity;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = HwFlowCtl;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  dbg_printf("Reconfiguring UART with %d baud, %d%c%d , %s flow control\n",
    baud, bits, parity_str, stop_bits, flow_control ? "" : "no");

  HAL_UART_DeInit(&huart2);

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }

  UART2_enable_rx_irq();
}

/*
volatile bool can1_rx_irq = false;
volatile bool can2_rx_irq = false;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (hfdcan == &hfdcan1) {
      can1_rx_irq = true;
    }
    if (hfdcan == &hfdcan2) {
      can2_rx_irq = true;
    }
  }
}
*/

int _read(int file, char *ptr, int len) {

}

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, ptr, len, 100);
  return len;
}

ring_buffer_t uart_ring_buffer;
ring_buffer_t virtual_uart_ring_buffer;

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Do MPU */
  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = (uint32_t)TX_Buffer;
  MPU_InitStruct.Size = MPU_REGION_SIZE_2KB;    
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer_userspace;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static FLASH_OBProgramInitTypeDef OBInit;

void disableCM4Autoboot() {
  OBInit.Banks     = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  dbg_printf("OBInit.USERConfig: %X\n", OBInit.USERConfig);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    dbg_printf("Changing option bytes\n");
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    if (HAL_FLASH_OB_Unlock() == HAL_OK)
      if (HAL_FLASH_Unlock() == HAL_OK)
        if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK)
          if (HAL_FLASH_OB_Launch() == HAL_OK)
            if (HAL_FLASH_OB_Lock() == HAL_OK)
              if (HAL_FLASH_Lock() == HAL_OK)
              {
                dbg_printf("Option bytes changed\n");
                dbg_printf("Requires rebooting\n");
                NVIC_SystemReset();
                return;
              }
    dbg_printf("Failed changing option bytes");
  }
}


int main(void) {

  MPU_Config();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  MX_USART2_UART_Init();

  PeriphCommonClock_Config();

  ring_buffer_init(&uart_ring_buffer);
  ring_buffer_init(&virtual_uart_ring_buffer);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM_Init();
  MX_RTC_Init();
  MX_SPI3_Init();

  //Initialize and start the ADCs
  adc_init();

#ifdef PORTENTA_DEBUG_WIRED
  // Enable SPI2 (Portenta only)
  MX_SPI2_Init();
#endif

  //Initilize CAN
  canInit();

  disableCM4Autoboot();

  memset((uint8_t*)TX_Buffer, 0, sizeof(TX_Buffer));
  memset((uint8_t*)RX_Buffer, 0, sizeof(RX_Buffer));

  struct complete_packet *tx_pkt = (struct complete_packet *)TX_Buffer;

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // IRQ PIN from H7 to M8
  // TODO: changeme when final HW is ready

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);

#ifndef PORTENTA_DEBUG_WIRED

  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Interrupt on CS LOW
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

#else

  // Enable LEDs (Portenta only)
  __HAL_RCC_GPIOK_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  // Interrupt on CS LOW
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#endif

  // Start DMA on SPI
  //HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)TX_Buffer,
  //                            (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);

  printf("Portenta X8 - STM32H7 companion fw - %s %s\n", __DATE__, __TIME__);

  int m4_app_valid = (((*(__IO uint32_t *) FLASH_BANK2_BASE) & 0xFF000000) == 0x10000000);

  if (m4_app_valid) {
    printf("Boot CM4\n");
    LL_RCC_ForceCM4Boot();
    int ret = serial_rpc_begin();
    printf("CM4 booted: %d\n", ret);
  }

  watchdog.Instance = IWDG1;
  watchdog.Init.Prescaler = IWDG_PRESCALER_16;
  watchdog.Init.Reload = (32000 * 2000) / (16 * 1000); /* 2000 ms */
  watchdog.Init.Window = (32000 * 2000) / (16 * 1000); /* 2000 ms */

  HAL_IWDG_Init(&watchdog);

#ifdef PORTENTA_DEBUG_WIRED
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 1);
#endif

  while (1) {

    __WFI();

    //serial_rpc_available();
    HAL_IWDG_Refresh(&watchdog);

    if (!ring_buffer_is_empty(&uart_ring_buffer)) {
        uint8_t temp_buf[1024];
        __disable_irq();
        int cnt = ring_buffer_dequeue_arr(&uart_ring_buffer, temp_buf, ring_buffer_num_items(&uart_ring_buffer));
				__enable_irq();
        enqueue_packet(PERIPH_UART, DATA, cnt, temp_buf);
    }

    if (!ring_buffer_is_empty(&virtual_uart_ring_buffer)) {
    		printf("ring_buffer_num_items: %d\n", ring_buffer_num_items(&virtual_uart_ring_buffer));
        uint8_t temp_buf[1024];
        __disable_irq();
        int cnt = ring_buffer_dequeue_arr(&virtual_uart_ring_buffer, temp_buf, ring_buffer_num_items(&virtual_uart_ring_buffer));
        __enable_irq();
        enqueue_packet(PERIPH_VIRTUAL_UART, DATA, cnt, temp_buf);
    }

    can_handle_data();

    if (transferState == TRANSFER_COMPLETE) {

#ifdef PORTENTA_DEBUG_WIRED
      HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 1);
#endif

      struct subpacket *rx_pkt_userspace =
          (struct subpacket *)RX_Buffer_userspace;

      while (rx_pkt_userspace->peripheral != 0xFF &&
             rx_pkt_userspace->peripheral != 0x00) {
        dbg_printf("Peripheral: %s Opcode: %X Size: %X\n  data: ",
        		to_peripheral_string(rx_pkt_userspace->peripheral), rx_pkt_userspace->opcode,
                rx_pkt_userspace->size);
        for (int i = 0; i < rx_pkt_userspace->size; i++) {
          dbg_printf("0x%02X ", *((&rx_pkt_userspace->raw_data) + i));
        }
        dbg_printf("\n");

        // do something useful with this packet
        dispatchPacket(rx_pkt_userspace->peripheral, rx_pkt_userspace->opcode,
        		rx_pkt_userspace->size, &(rx_pkt_userspace->raw_data));

        rx_pkt_userspace = (struct subpacket *)((uint8_t *)rx_pkt_userspace + 4 + rx_pkt_userspace->size);
      }
      transferState = TRANSFER_WAIT;
    }

    if (transferState == TRANSFER_ERROR) {
        dbg_printf("got transfer error, recovering\n");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
        transferState = TRANSFER_WAIT;
    }

/*
    if (trigger_irq) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
      trigger_irq = false;
    }
*/
  }
}

#ifdef PORTENTA_DEBUG_WIRED
void EXTI0_IRQHandler(void)
{
	if (get_data_amount) {
		HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)TX_Buffer,
		                                (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
#endif

void EXTI15_10_IRQHandler(void)
{
  if (get_data_amount) {
    HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)TX_Buffer,
                                    (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
  }
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

  struct complete_packet *rx_pkt = (struct complete_packet *)RX_Buffer;
  struct complete_packet *tx_pkt = (struct complete_packet *)TX_Buffer;

  if (get_data_amount) {

#ifdef PORTENTA_DEBUG_WIRED
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5, 0);
#endif

    data_amount = max(tx_pkt->size, rx_pkt->size);

    if (data_amount == 0) {
      return;
    }

    // reconfigure the DMA to actually receive the data
#ifndef PORTENTA_DEBUG_WIRED
    HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*)&(tx_pkt->data), (uint8_t*)&(rx_pkt->data), data_amount);
#else
    HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)&(tx_pkt->data), (uint8_t*)&(rx_pkt->data), data_amount);
#endif
    get_data_amount = false;

  } else {
    // real end of operation, pause DMA, memcpy stuff around and reenable DMA
    // HAL_SPI_DMAPause(&hspi1);

    transferState = TRANSFER_COMPLETE;

    memcpy((void *)RX_Buffer_userspace, &(rx_pkt->data), rx_pkt->size);

    // mark the next packet as invalid
    *((uint32_t*)((uint8_t *)RX_Buffer_userspace + rx_pkt->size)) = 0xFFFFFFFF; // INVALID;

    // clean the transfer buffer size to restart
    tx_pkt->size = 0;

#ifdef PORTENTA_DEBUG_WIRED
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_6, 0);
#endif

    get_data_amount = true;

    // HAL_SPI_DMAResume(&hspi1);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  transferState = TRANSFER_ERROR;

#ifndef PORTENTA_DEBUG_WIRED
  HAL_SPI_Abort(&hspi3);
#else
  HAL_SPI_Abort(&hspi2);
#endif

/*
  // Restart DMA
  #ifndef PORTENTA_DEBUG_WIRED
  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)TX_Buffer,
                                  (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
  #else
  HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)TX_Buffer,
                                  (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
  #endif
*/
}


void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // ATTENTION: make sure this matches the actual hardware configuration
#ifndef PORTENTA_DEBUG_WIRED
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
#else
  HAL_PWREx_ConfigSupply(PWR_SMPS_1V8_SUPPLIES_LDO);
#endif

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
}

static void MX_HRTIM_Init(void) {

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  hhrtim.Instance = HRTIM1;
  hhrtim.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim) != HAL_OK) {
    Error_Handler();
  }
  HAL_HRTIM_MspPostInit(&hhrtim);
}

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

static void MX_SPI2_Init(void) {

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_SPI3_Init(void) {

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
}

static uint8_t uart_rxbuf[1024];

void UART2_enable_rx_irq() {
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXFNE);
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_rxbuf, sizeof(uart_rxbuf));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  ring_buffer_queue_arr(&uart_ring_buffer, uart_rxbuf, Size);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_rxbuf, sizeof(uart_rxbuf));
}

static void MX_USART2_UART_Init(void) {

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_2) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_2) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }

/*
  UART_WakeUpTypeDef event = 
    { .WakeUpEvent = UART_WAKEUP_ON_STARTBIT };
  HAL_UARTEx_StopModeWakeUpSourceConfig(&huart2, event);
  HAL_UARTEx_EnableStopMode(&huart2);
*/
  UART2_enable_rx_irq();
}

static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

static void MX_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

void Error_Handler_Name(const char* name) {
  dbg_printf("Error_Handler called by %s\n", name);
  __disable_irq();
  while (1) {
  }
}
