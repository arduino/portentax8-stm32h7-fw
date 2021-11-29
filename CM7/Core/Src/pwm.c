#include "pwm.h"
#include "system.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "timer.h"

#define PWM_NUMBER    10

struct PWM_numbers {
  GPIO_TypeDef * port;
  uint16_t pin;
  uint16_t alternate;
};

struct PWM_numbers PWM_pinmap[] = {
  // GPIOs
  { GPIOC, GPIO_PIN_7,  GPIO_AF2_TIM3 },
  { GPIOA, GPIO_PIN_9,  GPIO_AF1_TIM1 },
  { GPIOA, GPIO_PIN_10, GPIO_AF1_TIM1 },
  { GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2 },
  { GPIOA, GPIO_PIN_11, GPIO_AF1_TIM1 },
  { GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4 },
  { GPIOA, GPIO_PIN_8,  GPIO_AF1_TIM1 },
  { GPIOC, GPIO_PIN_6,  GPIO_AF2_TIM3 },
  { GPIOC, GPIO_PIN_9,  GPIO_AF2_TIM3 },
  { GPIOC, GPIO_PIN_8,  GPIO_AF2_TIM3 },
};

struct PWM_timers {
  uint32_t index;
  uint32_t channel;
};

struct CAPTURE_numbers {
  TIM_TypeDef * timer_instance;
  uint32_t channel;
  uint32_t active_ch;
};

uint32_t capture_first = 0;
uint32_t capture_second = 0;
uint32_t capture_last = 0;

//uint32_t capture_value = 0;
uint32_t capture_duty = 0;
uint32_t capture_period = 0;

uint32_t capture_frequency = 0;

TIM_HandleTypeDef    htim1;
TIM_HandleTypeDef    htim2;
TIM_HandleTypeDef    htim3;
TIM_HandleTypeDef    htim4;

TIM_IC_InitTypeDef   sICConfig;

struct PWM_timers PWM_Timer_map[] = {
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
  { TIM3, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2 },   // PC7
  { TIM1, TIM_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_2 },   // PA9
  { TIM1, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3 },   // PA10
  { TIM2, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3 },   // PB10
  { TIM1, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4 },   // PA11
  { TIM4, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4 },   // PD15
  { TIM1, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1 },   // PA8
  { TIM3, TIM_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_1 },   // PC6
  { TIM3, TIM_CHANNEL_4, HAL_TIM_ACTIVE_CHANNEL_4 },   // PC9
  { TIM3, TIM_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_3 },   // PC8
};

void pwm_handler(uint8_t opcode, uint8_t *data, uint8_t size) {
  if (opcode & CAPTURE) {
    uint8_t channel = opcode & 0x0F;
    capturePwm(channel);
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

  pwm_timer_config(PWM_Timer_map[channel].index, PWM_Timer_map[channel].channel, &sConfig_Channel, &pTimeBaseCfg, timers, enable);
}

void configureTIM1(uint8_t channel) {

  htim1.Instance = TIM1;

  uint32_t timer_channel = CAPTURE_pinmap[channel].channel;

  // Initialize htim2 peripheral
  htim1.Init.Period            = 0xFFFF;
  htim1.Init.Prescaler         = 0;
  htim1.Init.ClockDivision     = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.RepetitionCounter = 0;
  if(HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_Init FAILED\n");
  }

  if(HAL_TIM_IC_ConfigChannel(&htim1, &sICConfig, timer_channel) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_ConfigChannel FAILED\n");
  }

  if(HAL_TIM_IC_Start_IT(&htim1, timer_channel) != HAL_OK) {
    dbg_printf("HAL_TIM_IC_Start_IT FAILED\n");
  }
}

void configureTIM2(uint8_t channel) {

  htim2.Instance = TIM2;

  uint32_t timer_channel = CAPTURE_pinmap[channel].channel;

  // Initialize htim2 peripheral
  htim2.Init.Period            = 0xFFFF;
  htim2.Init.Prescaler         = 0;
  htim2.Init.ClockDivision     = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.RepetitionCounter = 0;
  if(HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_Init FAILED\n");
  }

  if(HAL_TIM_IC_ConfigChannel(&htim2, &sICConfig, timer_channel) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_ConfigChannel FAILED\n");
  }

  if(HAL_TIM_IC_Start_IT(&htim2, timer_channel) != HAL_OK) {
    dbg_printf("HAL_TIM_IC_Start_IT FAILED\n");
  }
}

void configureTIM3(uint8_t channel) {

  htim3.Instance = TIM3;

  uint32_t timer_channel = CAPTURE_pinmap[channel].channel;

  // Initialize htim2 peripheral
  htim3.Init.Period            = 0xFFFF;
  htim3.Init.Prescaler         = 0;
  htim3.Init.ClockDivision     = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.RepetitionCounter = 0;
  if(HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_Init FAILED\n");
  }

  if(HAL_TIM_IC_ConfigChannel(&htim3, &sICConfig, timer_channel) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_ConfigChannel FAILED\n");
  }

  if(HAL_TIM_IC_Start_IT(&htim3, timer_channel) != HAL_OK) {
    dbg_printf("HAL_TIM_IC_Start_IT FAILED\n");
  }
}

void configureTIM4(uint8_t channel) {

  htim4.Instance = TIM4;

  uint32_t timer_channel = CAPTURE_pinmap[channel].channel;

  // Initialize htim2 peripheral
  htim4.Init.Period            = 0xFFFF;
  htim4.Init.Prescaler         = 0;
  htim4.Init.ClockDivision     = 0;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.RepetitionCounter = 0;
  if(HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_Init FAILED\n");
  }

  if(HAL_TIM_IC_ConfigChannel(&htim4, &sICConfig, timer_channel) != HAL_OK)
  {
    dbg_printf("HAL_TIM_IC_ConfigChannel FAILED\n");
  }

  if(HAL_TIM_IC_Start_IT(&htim4, timer_channel) != HAL_OK) {
    dbg_printf("HAL_TIM_IC_Start_IT FAILED\n");
  }
}

void capturePwm(uint8_t channel) {

  dbg_printf("PWM capture initialization...\n");

  // Configure the Input Capture channel
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;

  if (CAPTURE_pinmap[channel].timer_instance == TIM1) {
    configureTIM1(channel);
  } else if (CAPTURE_pinmap[channel].timer_instance == TIM2) {
    configureTIM2(channel);
  } else if (CAPTURE_pinmap[channel].timer_instance == TIM3) {
    configureTIM3(channel);
  } else if (CAPTURE_pinmap[channel].timer_instance == TIM4) {
    configureTIM4(channel);
  } else {
    Error_Handler();
  }

}

uint8_t interrupt_count = 0;
uint8_t first_edge_rising = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t tim_channel;

  uint8_t pwm_idx = 0xFF;

  for (int i=0; i<PWM_NUMBER; i++) {
    if (htim->Instance == CAPTURE_pinmap[i].timer_instance && htim->Channel == CAPTURE_pinmap[i].active_ch) {
      tim_channel = CAPTURE_pinmap[i].channel;
      pwm_idx = i;
      break;
    }
  }

  if (pwm_idx == 0xFF) {
    //No matching PWM configuration found!
    Error_Handler();
  } else {

    if(interrupt_count == 0)
    {
      /* Get the 1st Input Capture value */
      capture_first = HAL_TIM_ReadCapturedValue(htim, tim_channel);
      first_edge_rising = HAL_GPIO_ReadPin(PWM_pinmap[pwm_idx].port, PWM_pinmap[pwm_idx].pin);
      interrupt_count = 1;
    } else if (interrupt_count == 1)
    {
      capture_second = HAL_TIM_ReadCapturedValue(htim, tim_channel);
      interrupt_count = 2;
    }
    else if(interrupt_count == 2)
    {
      /* Get the 2nd Input Capture value */
      capture_last = HAL_TIM_ReadCapturedValue(htim, tim_channel); 

      /* Capture computation */
      if (capture_last > capture_first)
      {
        capture_period = (capture_last - capture_first); 
      }
      else if (capture_last < capture_first)
      {
        /* 0xFFFF is max TIM1_CCRx value */
        capture_period = ((0xFFFF - capture_first) + capture_last) + 1;
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
           measures */
        Error_Handler();
      }

      if (first_edge_rising) {
        if (capture_second > capture_first)
        {
          capture_duty = (capture_second - capture_first); 
        }
        else if (capture_second < capture_first)
        {
          /* 0xFFFF is max TIM1_CCRx value */
          capture_duty = ((0xFFFF - capture_first) + capture_second) + 1;
        }
        else
        {
          /* If capture values are equal, we have reached the limit of frequency
            measures */
          Error_Handler();
        }
      } else {
        if (capture_last > capture_second)
        {
          capture_duty = (capture_last - capture_second); 
        }
        else if (capture_last < capture_second)
        {
          // 0xFFFF is max TIM1_CCRx value
          capture_duty = ((0xFFFF - capture_second) + capture_last) + 1;
        }
        else
        {
          Error_Handler();
        }
      }
      
      /* Frequency computation: for this example TIMx (TIM1) is clocked by
         2*APB2Clk as APB2CLKDivider are set to RCC_APB2_DIV2 */      
      capture_frequency = 2*HAL_RCC_GetPCLK2Freq() / capture_period;

      uint32_t ns_factor = 5; //(1000000000 / (2*HAL_RCC_GetPCLK2Freq()));

      //Period in ns
      capture_period = ns_factor * capture_period;

      //Duty in ns
      capture_duty = ns_factor * capture_duty;

      interrupt_count = 0;

      HAL_TIM_IC_Stop_IT(htim, tim_channel);

      struct pwmCapture capture;
      capture.duty = capture_duty;
      capture.period = capture_period;

      enqueue_packet(PERIPH_PWM, CAPTURE, sizeof(capture), &capture);
    }
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
 
  /*##-1- Configure GPIOs #################################*/

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  for (int i=1; i<PWM_NUMBER; i++) {
    if (htim->Instance == CAPTURE_pinmap[i].timer_instance) {
      dbg_printf("Found match: index = %d\n", i);
      GPIO_InitStruct.Pin = PWM_pinmap[i].pin;
      GPIO_InitStruct.Alternate = PWM_pinmap[i].alternate;
      HAL_GPIO_Init(PWM_pinmap[i].port, &GPIO_InitStruct);
    }
  }
  
  /*##-2- Enable clocks and Configure the NVIC #########################################*/

  if(htim->Instance == TIM1) {

    /* Enable GPIO channels Clock */
    __HAL_RCC_TIM1_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 1);
    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

  } else if (htim->Instance == TIM2) {

    /* Enable GPIO channels Clock */
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

  } else if (htim->Instance == TIM3) {

    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

  } else if (htim->Instance == TIM4) {

    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

  }
}

void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}