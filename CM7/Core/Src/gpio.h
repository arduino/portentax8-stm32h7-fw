#include <inttypes.h>

#ifndef GPIO_H
#define GPIO_H

void gpio_init();

void configureGPIO(uint8_t opcode, uint16_t data);

void gpio_set_initial_config();

#endif  //GPIO_H