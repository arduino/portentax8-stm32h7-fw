#include "gpio.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

struct GPIO_numbers {
  GPIO_TypeDef * port;
  uint16_t pin;
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

static void MX_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

void gpio_init() {
  MX_GPIO_Init();
}

void configureGPIO(uint8_t opcode, uint16_t data) {
  enum Opcodes_GPIO action = opcode;

  uint8_t value = (data & 0xFF00) >> 8;
  uint8_t index = data & 0xFF;

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t response[2];

  switch (action) {
    case CONFIGURE:
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
      response[0] = index;
      response[1] = HAL_GPIO_ReadPin(GPIO_pinmap[index].port, GPIO_pinmap[index].pin);
      enqueue_packet(PERIPH_GPIO, opcode, sizeof(response), &response);
      dbg_printf("GPIO%d: READ %d\n", index, response[1]);
      break;
  }
}

void gpio_set_initial_config() {
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
}