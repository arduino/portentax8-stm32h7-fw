#include <inttypes.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#ifndef TIMER_H
#define TIMER_H

void timer_init();

void pwm_timer_config(uint32_t index, uint32_t channel,
                      HRTIM_SimplePWMChannelCfgTypeDef* pSimplePWMChannelCfg,
                      HRTIM_TimeBaseCfgTypeDef * pTimeBaseCfg,
                      uint32_t timers, bool enable);

#endif //TIMER_H