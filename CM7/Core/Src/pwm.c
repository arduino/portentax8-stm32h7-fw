#include "pwm.h"
#include "system.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "timer.h"

#define PWM_NUMBER    10

struct PWM_numbers {
  uint32_t index;
  uint32_t channel;
};

struct CAPTURE_numbers {
  TIM_TypeDef * timer_instance;
  uint32_t channel;
};

uint32_t capture_first = 0;
uint32_t capture_second = 0;
uint32_t capture_last = 0;

//uint32_t capture_value = 0;
uint32_t capture_duty = 0;
uint32_t capture_period = 0;

TIM_HandleTypeDef    TimHandle;
/*
TIM_HandleTypeDef    htim1;
TIM_HandleTypeDef    htim2;
TIM_HandleTypeDef    htim3;
TIM_HandleTypeDef    htim4;
*/
TIM_IC_InitTypeDef   sICConfig;

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

struct CAPTURE_numbers CAPTURE_pinmap[] = {
  // GPIOs
  { TIM3, TIM_CHANNEL_2 },   // PC7
  { TIM1, TIM_CHANNEL_2 },   // PA9
  { TIM1, TIM_CHANNEL_3 },   // PA10
  { TIM2, TIM_CHANNEL_3 },   // PG10
  { TIM1, TIM_CHANNEL_4 },   // PA11
  { TIM4, TIM_CHANNEL_4 },   // PD15
  { TIM1, TIM_CHANNEL_1 },   // PA8
  { TIM3, TIM_CHANNEL_1 },   // PC6
  { TIM3, TIM_CHANNEL_4 },   // PC9
  { TIM3, TIM_CHANNEL_3 },   // PC8
};

void pwm_handler(uint8_t opcode, uint8_t *data, uint8_t size) {
  if (opcode & CAPTURE) {
    uint8_t channel = opcode & 0x0F;
    captureFreq(channel);
  } else {
    uint8_t channel = opcode;
    struct pwmPacket config = *((struct pwmPacket*)data);
    configurePwm(channel, config.enable, config.polarity, config.duty, config.period);
  }
}

void pwm_init() {
  register_peripheral_callback(PERIPH_PWM, &pwm_handler);
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

  uint32_t timers = HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_C | HRTIM_TIMERUPDATE_D | HRTIM_TIMERUPDATE_E;

  pwm_timer_config(PWM_pinmap[channel].index, PWM_pinmap[channel].channel, &sConfig_Channel, &pTimeBaseCfg, timers, enable);
}

void captureFreq(uint8_t channel) {

  dbg_printf("PWM capture initialization...\n");

  TimHandle.Instance = CAPTURE_pinmap[channel].timer_instance;

  uint32_t timer_channel = CAPTURE_pinmap[channel].channel;

  // Initialize htim2 peripheral
  TimHandle.Init.Period            = 0xFFFF;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_Init FAILED\n");
  }

  // Configure the Input Capture channel
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, timer_channel) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_ConfigChannel FAILED\n");
  }

  if(HAL_TIM_IC_Start_IT(&TimHandle, timer_channel) != HAL_OK) {
    dbg_printf("HAL_TIM_IC_Start_IT FAILED\n");
  }
}

uint8_t interrupt_count = 0;
uint8_t first_edge_rising = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t tim_channel;
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    tim_channel = TIM_CHANNEL_1;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    tim_channel = TIM_CHANNEL_2;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    tim_channel = TIM_CHANNEL_3;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    tim_channel = TIM_CHANNEL_4;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_5) {
    tim_channel = TIM_CHANNEL_5;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_6) {
    tim_channel = TIM_CHANNEL_6;
  }
  /*
  for (int i=0; i<PWM_NUMBER; i++) {
    if (htim->Instance == CAPTURE_pinmap[i].timer_instance && CAPTURE_pinmap[i].channel == tim_channel) {

    }
  }
  */

  if (interrupt_count == 0) {
    //Register time at first rising edge
    capture_first = HAL_TIM_ReadCapturedValue(htim, tim_channel);
    //Check if first edge is rising
    first_edge_rising = HAL_GPIO_ReadPin(GPIOA, 8);
  }
  if (interrupt_count == 1) {
    //Register time at second rising edge
    capture_second = HAL_TIM_ReadCapturedValue(htim, tim_channel);
  }
  if (interrupt_count == 2) {
    //Register time at second rising edge
    capture_last = HAL_TIM_ReadCapturedValue(htim, tim_channel);
  }
  
  interrupt_count++;

  if (interrupt_count == 3) {
    capture_period = (capture_last - capture_first)/(2*HAL_RCC_GetPCLK2Freq());
    //capture_freq = 2*HAL_RCC_GetPCLK2Freq()/period;
    if (first_edge_rising) {
      capture_duty = (capture_second - capture_first)/(2*HAL_RCC_GetPCLK2Freq());
    } else {
      capture_duty = (capture_last - capture_second)/(2*HAL_RCC_GetPCLK2Freq());
    }
    interrupt_count = 0;

    HAL_TIM_IC_Stop_IT(htim, tim_channel);

    struct pwmCapture capture;
    capture.duty = capture_duty;
    capture.period = capture_period;

    enqueue_packet(PERIPH_PWM, CAPTURE, sizeof(capture), &capture);
  }
}

void pwm_capture_read_data() {
  /*
  if (capture_value) {
    dbg_printf("capture value: %d\n", capture_value);
  }
  */
  if (capture_duty) {
    dbg_printf("capture duty: %d\n", capture_duty);
  }
  if (capture_period) {
    dbg_printf("capture freq: %d\n", capture_period);
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
 
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  
  /* Enable GPIO channels Clock */
  //TIMx_CHANNEL_GPIO_PORT();

  /* Configure  (TIMx_Channel) in Alternate function, push-pull and high speed */
  
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  
  /*##-2- Configure the NVIC for TIMx #########################################*/

  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 1);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}