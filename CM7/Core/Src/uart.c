#include "uart.h"
#include "peripherals.h"
#include "main.h"
#include "ringbuffer.h"

UART_HandleTypeDef huart2;

ring_buffer_t uart_ring_buffer;
ring_buffer_t virtual_uart_ring_buffer;

static uint8_t uart_rxbuf[1024];

struct __attribute__((packed, aligned(4))) uartPacket {
  uint8_t bits: 4;
  uint8_t stop_bits: 2;
  uint8_t parity: 2;
  uint8_t flow_control: 1;
  uint32_t baud: 23;
};

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, ptr, len, 100);
  return len;
}

int _read(int file, char *ptr, int len) {

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


void uart_init() {
  MX_USART2_UART_Init();

  ring_buffer_init(&uart_ring_buffer);
  ring_buffer_init(&virtual_uart_ring_buffer);
}

int uart_write(uint8_t *data, uint16_t size, uint32_t timeout) {
  return HAL_UART_Transmit(&huart2, data, size, timeout);
}

int uart_data_available() {
  return !ring_buffer_is_empty(&uart_ring_buffer);
}

void uart_handle_data() {
  uint8_t temp_buf[1024];
  __disable_irq();
  int cnt = ring_buffer_dequeue_arr(&uart_ring_buffer, temp_buf, ring_buffer_num_items(&uart_ring_buffer));
  __enable_irq();
  enqueue_packet(PERIPH_UART, DATA, cnt, temp_buf);
}

int virtual_uart_data_available() {
  return !ring_buffer_is_empty(&virtual_uart_ring_buffer);
}

void virtual_uart_handle_data() {
  uint8_t temp_buf[1024];
  __disable_irq();
  int cnt = ring_buffer_dequeue_arr(&virtual_uart_ring_buffer, temp_buf, ring_buffer_num_items(&virtual_uart_ring_buffer));
  __enable_irq();
  enqueue_packet(PERIPH_VIRTUAL_UART, DATA, cnt, temp_buf);
}

void UART2_enable_rx_irq() {
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXFNE);
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_rxbuf, sizeof(uart_rxbuf));
}


void uart_configure(uint8_t *data) {

  struct uartPacket config = *((struct uartPacket*)data);

  //HAL_UART_DeInit(&huart2);

  uint32_t WordLength;
  uint32_t Parity;
  uint32_t StopBits;
  uint32_t HwFlowCtl = UART_HWCONTROL_NONE;

  switch (config.bits) {
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

  switch (config.parity) {
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

  switch (config.stop_bits) {
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

  if (config.flow_control) {
    HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  }

  if (huart2.Init.BaudRate == config.baud &&
      huart2.Init.WordLength == WordLength &&
      huart2.Init.StopBits == StopBits &&
      huart2.Init.Parity == Parity &&
      huart2.Init.HwFlowCtl == HwFlowCtl) {
      return;
  }

  huart2.Init.BaudRate = config.baud;
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
    config.baud, config.bits, parity_str, config.stop_bits, config.flow_control ? "" : "no");

  HAL_UART_DeInit(&huart2);

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }

  UART2_enable_rx_irq();
}