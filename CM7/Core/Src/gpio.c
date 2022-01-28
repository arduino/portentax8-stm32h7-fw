#include "gpio.h"
#include "main.h"
#include "system.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

struct GPIO_numbers {
  GPIO_TypeDef * port;
  uint16_t pin;
};

struct IRQ_numbers {
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

struct IRQ_numbers IRQ_pinmap[16];

static void MX_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

#define GPIO_MODE_IN_RE         0x01   /*!< Input interrupt rising edge */
#define GPIO_MODE_IN_FE         0x02   /*!< Input interrupt falling edge */
#define GPIO_MODE_IN_AH         0x04   /*!< Input interrupt active high */
#define GPIO_MODE_IN_AL         0x08   /*!< Input interrupt active low */

static void handle_irq() {
  uint32_t pr = EXTI->PR1;
  uint8_t index = 0;
  while (pr != 0) {
    if (pr & 0x1) {
      uint8_t irq = IRQ_pinmap[index].pin;
      enqueue_packet(PERIPH_GPIO, IRQ_SIGNAL, sizeof(irq), &irq);
      HAL_GPIO_EXTI_IRQHandler(1 << index);
    }
    pr >>= 1;
    index++;
  }
}

static void disable_irq(uint8_t pin) {
  if (pin == GPIO_PIN_0) {
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  }
  else if (pin == GPIO_PIN_1) {
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  }
  else if (pin == GPIO_PIN_2) {
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  }
  else if (pin == GPIO_PIN_3) {
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  }
  else if (pin == GPIO_PIN_4) {
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  }
  else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  }
  else {
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  }
}

static void enable_irq(uint8_t pin) {
  if (pin == GPIO_PIN_0) {
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetVector(EXTI0_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_1) {
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetVector(EXTI1_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_2) {
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_SetVector(EXTI2_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_3) {
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_SetVector(EXTI3_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_4) {
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_SetVector(EXTI4_IRQn, (uint32_t)&handle_irq);
  } else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetVector(EXTI9_5_IRQn, (uint32_t)&handle_irq);
  } else {
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetVector(EXTI15_10_IRQn, (uint32_t)&handle_irq);
  }
}

static uint8_t GPIO_PIN_to_index(uint32_t pin) {
  uint8_t index = 0;
  while (pin >>= 1) {
    index++;
  }
  return index;
}

void gpio_handler(uint8_t opcode, uint8_t *pdata, uint16_t size) {
  uint16_t data = *((uint16_t*)pdata);
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
    case IRQ_TYPE:
      GPIO_InitStruct.Pin = GPIO_pinmap[index].pin;
      GPIO_InitStruct.Mode = (value == GPIO_MODE_IN_RE || value == GPIO_MODE_IN_AH ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING) ;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_pinmap[index].port, &GPIO_InitStruct);
      enable_irq(GPIO_pinmap[index].pin);
      IRQ_pinmap[GPIO_PIN_to_index(GPIO_InitStruct.Pin)].pin = index;
      break;
    case IRQ_ENABLE:
      if (value == 1) {
        enable_irq(GPIO_pinmap[index].pin);
      } else {
        disable_irq(GPIO_pinmap[index].pin);
      }
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
    case IRQ_ACK:
      //do nothing
      break;
  }
}

void gpio_init() {
  MX_GPIO_Init();

  register_peripheral_callback(PERIPH_GPIO, &gpio_handler);
}

void gpio_set_initial_config() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // IRQ PIN from H7 to M8
  // TODO: changeme when final HW is ready

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Interrupt on CS LOW
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}