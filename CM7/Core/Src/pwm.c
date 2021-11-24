#include "pwm.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"

struct PWM_numbers {
  uint32_t index;
  uint32_t channel;
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