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

#include "gpio.h"
#include "main.h"
#include "system.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

struct GPIO_numbers {
  GPIO_TypeDef * port;
  uint16_t pin;
};

struct IRQ_numbers {
  uint16_t pin;
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

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

static volatile uint16_t int_event_flags = 0;

/**************************************************************************************
 * INTERNAL FUNCTION DECLARATION
 **************************************************************************************/

static void gpio_disable_irq(uint8_t pin);
static void gpio_enable_irq(uint8_t pin);
static void gpio_set_handler(uint8_t pin);

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void MX_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
}

#define GPIO_MODE_IN_RE         0x01   /*!< Input interrupt rising edge */
#define GPIO_MODE_IN_FE         0x02   /*!< Input interrupt falling edge */
#define GPIO_MODE_IN_AH         0x04   /*!< Input interrupt active high */
#define GPIO_MODE_IN_AL         0x08   /*!< Input interrupt active low */

volatile uint16_t ack_system = 0;

static void handle_irq() {
  uint32_t pr = EXTI->PR1;
  uint8_t index = 0;
  while (pr != 0) {
    if (pr & 0x1) {
      /* Set the flag variable which leads to a transmission
       * of a interrupt event within gpio_handle_data.
       */
      int_event_flags |= (1 << index);
      /* Clear interrupt flag for this specific GPIO interrupt. */
      HAL_GPIO_EXTI_IRQHandler(1 << index);
      /* Disable the interrupt that just fired in order to
       * prevent it from immediately firing again.
       */
      gpio_disable_irq(GPIO_pinmap[index].pin);
    }
    pr >>= 1;
    index++;
  }
}

static void gpio_disable_irq(uint8_t pin) {
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
}

static void gpio_enable_irq(uint8_t pin) {
  if (pin == GPIO_PIN_0) {
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  } else if (pin == GPIO_PIN_1) {
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  } else if (pin == GPIO_PIN_2) {
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  } else if (pin == GPIO_PIN_3) {
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  } else if (pin == GPIO_PIN_4) {
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  } else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
}

static void gpio_set_handler(uint8_t pin)
{
  if (pin == GPIO_PIN_0) {
    NVIC_SetVector(EXTI0_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_1) {
    NVIC_SetVector(EXTI1_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_2) {
    NVIC_SetVector(EXTI2_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_3) {
    NVIC_SetVector(EXTI3_IRQn, (uint32_t)&handle_irq);
  } else if (pin == GPIO_PIN_4) {
    NVIC_SetVector(EXTI4_IRQn, (uint32_t)&handle_irq);
  } else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    NVIC_SetVector(EXTI9_5_IRQn, (uint32_t)&handle_irq);
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
      GPIO_InitStruct.Mode = ((value == GPIO_MODE_IN_RE) || (value == GPIO_MODE_IN_AH)) ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_pinmap[index].port, &GPIO_InitStruct);
      IRQ_pinmap[GPIO_PIN_to_index(GPIO_InitStruct.Pin)].pin = index;
      dbg_printf("GPIO%d: IRQ_TYPE %d\n", index, value);
      break;
    case IRQ_ENABLE:
      if (value == 1) {
        gpio_set_handler(GPIO_pinmap[index].pin);
        gpio_enable_irq(GPIO_pinmap[index].pin);
      } else {
        gpio_disable_irq(GPIO_pinmap[index].pin);
      }
      dbg_printf("GPIO%d: IRQ_ENABLE %d\n", index, value);
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
    case IRQ_SIGNAL:
      // do nothing;
      dbg_printf("GPIO%d: IRQ_SIGNAL %d\n", index, value);
      break;
    case IRQ_ACK:
      // Clear busy bit
      ack_system &= ~(1 << index);
      //__disable_irq();
      /* Clear again interrupt flag for this specific GPIO interrupt.
       * This serves to make sure that we are not returning straight to
       * another interrupt when calling gpio_enable_irq in the line below.
       */
      HAL_GPIO_EXTI_IRQHandler(1 << index);
      /* Re-enable the interrupt that was disabled within
       * handle_irq to prevent firing of another interrupt
       * until this one has been signalled to the application.
       */
      gpio_enable_irq(GPIO_pinmap[index].pin);
      //__enable_irq();
      dbg_printf("GPIO%d: IRQ_ACK %d ack_system %x\n", index, value, ack_system);
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

void gpio_handle_data()
{
  /* Take a threadsafe copy of the interrupt flags. */
  __disable_irq();
  uint16_t const copy_int_event_flags = int_event_flags;
  __enable_irq();

  /* We have a total of 10 external interrupts. */
  for (uint8_t index = 0; index < 10; index++)
  {
    /* Check whether or not an external interrupt has occured. */
    if (copy_int_event_flags & (1 << index))
    {
      if(!(ack_system & (1 << index))) { /* Enqueue packet only if not busy */
        /* Send information to the AP. */
        uint8_t irq_pin = IRQ_pinmap[index].pin;
        enqueue_packet(PERIPH_GPIO, IRQ_SIGNAL, sizeof(irq_pin), &irq_pin);
        ack_system |= (1 << index); /* Add busy bit to our irq pin */
        __disable_irq();
        int_event_flags &= ~(1 << index); /*Clear this flag */
        __enable_irq();
      }
    }
  }
}
