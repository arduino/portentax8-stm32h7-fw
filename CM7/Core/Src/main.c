#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "ringbuffer.h"
#include "can_api.h"

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

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
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_HRTIM_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);

//#undef DEBUG

//#define PORTENTA_DEBUG_WIRED

#ifdef DEBUG
#define dbg_printf(...)	printf(__VA_ARGS__)
#else
#define dbg_printf(...)
#endif

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

__attribute__((section("dma"))) volatile uint8_t TX_Buffer[SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"))) volatile uint8_t RX_Buffer[SPI_DMA_BUFFER_SIZE];

__attribute__((section("dma"))) volatile uint8_t RX_Buffer_userspace[SPI_DMA_BUFFER_SIZE];

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

enum Peripherals {
  PERIPH_H7 = 0x00,
  PERIPH_ADC = 0x01,
	PERIPH_PWM = 0x02,
	PERIPH_FDCAN1 = 0x03,
	PERIPH_FDCAN2 = 0x04,
	PERIPH_UART = 0x05,
	PERIPH_RTC = 0x06,
  PERIPH_GPIO = 0x07,
};

const char* to_peripheral_string(enum Peripherals peripheral) {
	switch (peripheral) {
		case PERIPH_ADC:
			return "ADC";
		case PERIPH_PWM:
			return "PWM";
		case PERIPH_FDCAN1:
			return "FDCAN1";
		case PERIPH_FDCAN2:
			return "FDCAN2";
		case PERIPH_UART:
			return "UART";
		case PERIPH_RTC:
			return "RTC";
		case PERIPH_GPIO:
			return "GPIO";
		default:
			return "UNKNOWN";
	}
}

enum Opcodes {
	CONFIGURE = 0x10,
	DATA = 0x01,
  FW_VERSION = 0x10,
};

enum Opcodes_UART {
	GET_LINESTATE = 0x20,
};

enum Opcodes_RTC {
	SET_DATE = 0x01,
	GET_DATE = 0x02,
	SET_ALARM = 0x11,
	GET_ALARM = 0x12,
};

enum Opcodes_GPIO {
	DIRECTION = 0x10,
	WRITE = 0x20,
	READ = 0x30,
};

enum AnalogPins {
	A0 = 0x1,
	A1,
	A2,
	A3,
	A4,
	A5,
	A6,
	A7,
};

struct GPIO_numbers {
  GPIO_TypeDef * port;
  uint16_t pin;
};

struct PWM_numbers {
  uint32_t index;
  uint32_t channel;
};

struct ADC_numbers {
  ADC_HandleTypeDef* peripheral;
  uint32_t channel;
};

struct ADC_numbers ADC_pinmap[] = {
  { NULL, 0 },
  { &hadc1, ADC_CHANNEL_2 },
  { &hadc2, ADC_CHANNEL_3 },
  { &hadc2, ADC_CHANNEL_2 },
  { &hadc2, ADC_CHANNEL_5 },
  { &hadc2, ADC_CHANNEL_4 },
  { &hadc3, ADC_CHANNEL_3 },
  { &hadc3, ADC_CHANNEL_2 },
  { &hadc3, ADC_CHANNEL_4 },
};

struct GPIO_numbers GPIO_pinmap[] = {
  // GPIOs
  { GPIOF, GPIO_PIN_8 },
  { GPIOF, GPIO_PIN_6 },
  { GPIOF, GPIO_PIN_3 },
  { GPIOF, GPIO_PIN_4 },
  { GPIOF, GPIO_PIN_12 },
  { GPIOE, GPIO_PIN_10 },
  { GPIOE, GPIO_PIN_11 },
  // ADCs
  { GPIOF, GPIO_PIN_11 },
  { GPIOA, GPIO_PIN_6 },
  { GPIOF, GPIO_PIN_13 },
  { GPIOB, GPIO_PIN_1 },
  { GPIOC, GPIO_PIN_4 },
  { GPIOF, GPIO_PIN_7 },
  { GPIOF, GPIO_PIN_9 },
  { GPIOF, GPIO_PIN_5 },
  // FDCAN1
  { GPIOD, GPIO_PIN_1 },
  { GPIOD, GPIO_PIN_0 },
  // FDCAN1
  { GPIOB, GPIO_PIN_6 },
  { GPIOB, GPIO_PIN_5 },
  // USART2
  { GPIOD, GPIO_PIN_5 },
  { GPIOD, GPIO_PIN_6 },
  { GPIOD, GPIO_PIN_4 },
  { GPIOD, GPIO_PIN_3 },
  // PWM
  { GPIOC, GPIO_PIN_7 },
  { GPIOA, GPIO_PIN_9 },
  { GPIOA, GPIO_PIN_10 },
  { GPIOB, GPIO_PIN_10 },
  { GPIOA, GPIO_PIN_11 },
  { GPIOD, GPIO_PIN_15 },
  { GPIOA, GPIO_PIN_8 },
  { GPIOC, GPIO_PIN_6 },
  { GPIOA, GPIO_PIN_12 },
  { GPIOC, GPIO_PIN_8 },
};

struct PWM_numbers PWM_pinmap[] = {
  // GPIOs
  { HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2 },
  { HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1 },
  { HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2 },
  { HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2 },
  { HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1 },
  { HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1 },
  { HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2 },
  { HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1 },
  { HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD2 },
  { HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1 },
};

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

void enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void* data) {

/*
  int timeout = 100000;
	// don't feed data in the middle of a transmission
	while (get_data_amount == false && timeout > 0) {
		// wait for the DMA interrupt to be over
    timeout--;
	}
*/

  while (get_data_amount == false) {
    // wait for the DMA interrupt to be over
  }

	__disable_irq();
	struct complete_packet *tx_pkt = (struct complete_packet *)TX_Buffer;
	uint16_t offset = tx_pkt->size;
	if (offset + size > sizeof(TX_Buffer)) {
		goto cleanup;
	}
	struct subpacket pkt;
	pkt.peripheral = peripheral;
	pkt.opcode = opcode;
	pkt.size = size;
	memcpy((uint8_t*)&(tx_pkt->data) + offset, &pkt, 4);
	memcpy((uint8_t*)&(tx_pkt->data) + offset + 4, data, size);
	tx_pkt->size += 4 + size;
	tx_pkt->checksum = tx_pkt->size ^ 0x5555;

  dbg_printf("Enqueued packet for peripheral: %s Opcode: %X Size: %X\n  data: ",
      to_peripheral_string(peripheral), opcode, size);

  for (int i = 0; i < size; i++) {
    dbg_printf("0x%02X ", *(((uint8_t*)data) + i));
  }
  dbg_printf("\n");

cleanup:
	__enable_irq();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);

  //trigger_irq = true;
}

uint16_t get_ADC_value(enum AnalogPins name) {
	ADC_ChannelConfTypeDef conf = {0};
	ADC_HandleTypeDef* peripheral;

	conf.Rank = ADC_REGULAR_RANK_1;
	conf.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	conf.SingleDiff = ADC_SINGLE_ENDED;
	conf.OffsetNumber = ADC_OFFSET_NONE;

  conf.Channel = ADC_pinmap[name].channel;
  peripheral = ADC_pinmap[name].peripheral;

  HAL_ADC_ConfigChannel(peripheral, &conf);
  HAL_ADC_Start(peripheral);
  HAL_ADC_PollForConversion(peripheral, 10);
  uint16_t value = HAL_ADC_GetValue(peripheral);
  HAL_ADC_Stop(peripheral);

  dbg_printf("ADC%d: %d\n", name-1, value);

  enqueue_packet(PERIPH_ADC, name, sizeof(value), &value);
}

void configureGPIO(uint8_t opcode, uint16_t data) {
  enum Opcodes_GPIO action = opcode;

  uint8_t value = (data & 0xFF00) >> 8;
  uint8_t index = data & 0xFF;

  switch (action) {
    case CONFIGURE:
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_pinmap[index].pin;
      GPIO_InitStruct.Mode = value;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_pinmap[index].port, &GPIO_InitStruct);
      dbg_printf("GPIO%d: CONFIGURE %d\n", index, value);
      break;
    case WRITE:
      HAL_GPIO_WritePin(GPIO_pinmap[index].port, GPIO_pinmap[index].pin, value);
      dbg_printf("GPIO%d: WRITE %d\n", index, value);
      break;
    case READ:
      uint8_t value[2];
      value[0] = index;
      value[1] = HAL_GPIO_ReadPin(GPIO_pinmap[index].port, GPIO_pinmap[index].pin);
      enqueue_packet(PERIPH_GPIO, opcode, sizeof(value), &value);
      dbg_printf("GPIO%d: READ %d\n", index, value[1]);
      break;
  }
}

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

uint16_t adc_sample_rate = 0;

void configureFDCAN(uint8_t peripheral, void* data) {
  //HAL_FDCAN_ConfigFilter(&_hfdcan1, filterDef);
  //HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt);
}

void dispatchPacket(uint8_t peripheral, uint8_t opcode, uint16_t size, uint8_t* data) {
	switch (peripheral) {
	case PERIPH_ADC: {
		if (opcode == CONFIGURE) {
			adc_sample_rate = *((uint16_t*)data);
			dbg_printf("Setting ADC samplerate to %d milliseconds\n", adc_sample_rate);
      return;
		} else {
      // opcode == channel
      get_ADC_value(opcode);
    }
		break;
	}
	case PERIPH_PWM: {
		uint8_t channel = opcode;
		struct pwmPacket config = *((struct pwmPacket*)data);
		configurePwm(channel, config.enable, config.polarity, config.duty, config.period);
		break;
	}
  case PERIPH_GPIO: {
      configureGPIO(opcode, *((uint16_t*)data));
    break;
  }
  case PERIPH_FDCAN1:
  case PERIPH_FDCAN2: {
      if (opcode == CONFIGURE) {
        configureFDCAN(peripheral, data);
        break;
      }
      if (peripheral == PERIPH_FDCAN1) {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, (FDCAN_TxHeaderTypeDef*)data, data + sizeof(FDCAN_TxHeaderTypeDef));
      }
      if (peripheral == PERIPH_FDCAN2) {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, (FDCAN_TxHeaderTypeDef*)data, data + sizeof(FDCAN_TxHeaderTypeDef));
      }
    break;
  }
  case PERIPH_RTC: {
    doRTCStuff(opcode, (struct rtc_time*)data);
    break;
  }
  case PERIPH_UART: {
    if (opcode == CONFIGURE) {
      struct uartPacket config = *((struct uartPacket*)data);
      configureUart(config.baud, config.bits, config.parity, config.stop_bits, config.flow_control);
    } else {
      // can only write(), read() is irq driven
      HAL_UART_Transmit(&huart2, data, size, 0xFFFFFFFF);
      //HAL_UART_Transmit_IT(&huart2, data, size);
    }
    break;
  }
  case PERIPH_H7: {
    if (opcode == FW_VERSION) {
      writeVersion();
    }
    break;
  }
	}
}

int _read(int file, char *ptr, int len) {

}

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, ptr, len, 100);
  return len;
}

long adc_sample_rate_last_tick = 0;
ring_buffer_t uart_ring_buffer;

int main(void) {

  int32_t timeout;

  SCB_EnableICache();
  //SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  PeriphCommonClock_Config();

  ring_buffer_init(&uart_ring_buffer);

/*
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0);
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if (timeout < 0) {
    Error_Handler();
  }
*/

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_HRTIM_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();

#ifdef PORTENTA_DEBUG_WIRED
  // Enable SPI2 (Portenta only)
  MX_SPI2_Init();
#endif

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  can_t fdcan_1;
  can_t fdcan_2;
  can_init_freq_direct(&fdcan_1, CAN_1, 500000);
  can_init_freq_direct(&fdcan_2, CAN_2, 500000);

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

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

  watchdog.Instance = IWDG1;
  watchdog.Init.Prescaler = IWDG_PRESCALER_16;
  watchdog.Init.Reload = (32000 * 2000) / (16 * 1000); /* 2000 ms */
  watchdog.Init.Window = (32000 * 2000) / (16 * 1000); /* 2000 ms */

  HAL_IWDG_Init(&watchdog);


#ifdef PORTENTA_DEBUG_WIRED
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 1);
#endif

  while (1) {

    //__WFI();
    HAL_IWDG_Refresh(&watchdog);

    if (!ring_buffer_is_empty(&uart_ring_buffer)) {
        uint8_t temp_buf[1024];
        int cnt = ring_buffer_dequeue_arr(&uart_ring_buffer, temp_buf, ring_buffer_num_items(&uart_ring_buffer));
        enqueue_packet(PERIPH_UART, DATA, cnt, temp_buf);
    }

    if (HAL_FDCAN_IsRxBufferMessageAvailable(&hfdcan1, 0)) {
      FDCAN_RxHeaderTypeDef _rxHeader;
      uint8_t _rxData[64 + sizeof(_rxHeader)];
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &_rxHeader, _rxData + sizeof(_rxHeader));
      enqueue_packet(PERIPH_FDCAN1, DATA, _rxHeader.DataLength + sizeof(_rxHeader), _rxData);
    }
    if (HAL_FDCAN_IsRxBufferMessageAvailable(&hfdcan2, 0)) {
      FDCAN_RxHeaderTypeDef _rxHeader;
      uint8_t _rxData[64 + sizeof(_rxHeader)];
      HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &_rxHeader, _rxData + sizeof(_rxHeader));
      enqueue_packet(PERIPH_FDCAN1, DATA, _rxHeader.DataLength + sizeof(_rxHeader), _rxData);
    }

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

  //SCB_InvalidateDCache_by_Addr((uint32_t *)RX_Buffer, SPI_DMA_BUFFER_SIZE);
  //SCB_InvalidateDCache_by_Addr((uint32_t *)TX_Buffer, SPI_DMA_BUFFER_SIZE);

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

  #ifndef PORTENTA_DEBUG_WIRED
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }
  #endif

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void) {

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC2_Init(void) {

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC3_Init(void) {

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_FDCAN1_Init(void) {

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_FDCAN2_Init(void) {

  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
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
