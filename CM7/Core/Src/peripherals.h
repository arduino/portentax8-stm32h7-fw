#include <inttypes.h>

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

enum Peripherals {
  PERIPH_ADC = 0x01,
	PERIPH_PWM = 0x02,
	PERIPH_FDCAN1 = 0x03,
	PERIPH_FDCAN2 = 0x04,
	PERIPH_UART = 0x05,
	PERIPH_RTC = 0x06,
  PERIPH_GPIO = 0x07,
  PERIPH_M4 = 0x08,
  PERIPH_H7 = 0x09,
  PERIPH_VIRTUAL_UART = 0x0A,
};


enum Opcodes {
	CONFIGURE = 0x10,
	DATA = 0x01,
};

enum Opcodes_H7 {
  FW_VERSION = 0x10,
  BOOT_M4 = 0x77,
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

enum Opcodes_CAN {
  CAN_FILTER = 0x50,
};

enum Opcodes_GPIO {
	DIRECTION = 0x10,
	WRITE = 0x20,
	READ = 0x30,
};

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

const char* to_peripheral_string(enum Peripherals peripheral);

void configureGPIO(uint8_t opcode, uint16_t data);

void enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void* data);

void dispatchPacket(uint8_t peripheral, uint8_t opcode, uint16_t size, uint8_t* data);

void PeriphCommonClock_Config(void);

#endif //PERIPHERALS_H