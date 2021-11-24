#include "peripherals.h"
#include "stm32h7xx_hal.h"
#include "can_api.h"
#include "uart.h"
#include "pwm.h"
#include "main.h"

const char* to_peripheral_string(enum Peripherals peripheral) {
	switch (peripheral) {
		case PERIPH_ADC:
			return "ADC";
		case PERIPH_PWM:
			return "PWM";
		case PERIPH_FDCAN1:
			return "FDCAN1";
		case PERIPH_FDCAN2:
			return "FDCAN2";
		case PERIPH_UART:
			return "UART";
		case PERIPH_RTC:
			return "RTC";
    case PERIPH_GPIO:
      return "GPIO";
    case PERIPH_VIRTUAL_UART:
      return "VIRTUAL_UART";
		default:
			return "UNKNOWN";
	}
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

void enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void* data) {

/*
  int timeout = 100000;
	// don't feed data in the middle of a transmission
	while (get_data_amount == false && timeout > 0) {
		// wait for the DMA interrupt to be over
    timeout--;
	}
*/

  while (get_data_amount == false) {
    // wait for the DMA interrupt to be over
  }

	__disable_irq();
	struct complete_packet *tx_pkt = (struct complete_packet *)TX_Buffer;
	uint16_t offset = tx_pkt->size;
	if (offset + size > sizeof(TX_Buffer)) {
		goto cleanup;
	}
	struct subpacket pkt;
	pkt.peripheral = peripheral;
	pkt.opcode = opcode;
	pkt.size = size;
	memcpy((uint8_t*)&(tx_pkt->data) + offset, &pkt, 4);
	memcpy((uint8_t*)&(tx_pkt->data) + offset + 4, data, size);
	tx_pkt->size += 4 + size;
	tx_pkt->checksum = tx_pkt->size ^ 0x5555;

  dbg_printf("Enqueued packet for peripheral: %s Opcode: %X Size: %X\n  data: ",
      to_peripheral_string(peripheral), opcode, size);

  for (int i = 0; i < size; i++) {
    dbg_printf("0x%02X ", *(((uint8_t*)data) + i));
  }
  dbg_printf("\n");

cleanup:
	__enable_irq();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);

  //trigger_irq = true;
}

uint16_t adc_sample_rate = 0;

void dispatchPacket(uint8_t peripheral, uint8_t opcode, uint16_t size, uint8_t* data) {
	switch (peripheral) {
	case PERIPH_ADC: {
		if (opcode == CONFIGURE) {
			adc_sample_rate = *((uint16_t*)data);
			dbg_printf("Setting ADC samplerate to %d milliseconds\n", adc_sample_rate);
      return;
		} else {
      // opcode == channel
      get_ADC_value(opcode);
    }
		break;
	}
	case PERIPH_PWM: {
		uint8_t channel = opcode;
		struct pwmPacket config = *((struct pwmPacket*)data);
		configurePwm(channel, config.enable, config.polarity, config.duty, config.period);
		break;
	}
  case PERIPH_GPIO: {
    configureGPIO(opcode, *((uint16_t*)data));
    break;
  }
  case PERIPH_FDCAN1:
  case PERIPH_FDCAN2: {
    if (opcode == CONFIGURE) {
      configureFDCAN(peripheral, data);
      break;
    }

    if (opcode == CAN_FILTER) {
      uint32_t* info = (uint32_t*)data;
      CANFormat format = info[1] < 0x800 ? CANStandard : CANExtended;
      return canFilter(peripheral, info[1], info[2], format, info[0])
      break;
    }

    CAN_Message msg;
    msg.type = CANData;
    msg.format = CANStandard;
    memcpy(&msg, data, size);

    dbg_printf("Sending CAN message to %x, size %d, content[0]=0x%02X\n",
      msg.id, msg.len, msg.data[0]);

    if (msg.id > 0x7FF) {
      msg.format = CANExtended;
    }

    int ret = canWrite(peripheral, msg, 0);
    if (ret == 0) {
      canReset(peripheral);
    }
    break;
  }
  case PERIPH_RTC: {
    doRTCStuff(opcode, (struct rtc_time*)data);
    break;
  }
  case PERIPH_UART: {
    if (opcode == CONFIGURE) {
      uart_configure(data);
    } else {
      // can only write(), read() is irq driven
      uart_write(data, size, 0xFFFFFFFF);
      //HAL_UART_Transmit_IT(&huart2, data, size);
    }
    break;
  }
  case PERIPH_H7: {
    if (opcode == FW_VERSION) {
      writeVersion();
    }
    break;
  }
  case PERIPH_VIRTUAL_UART: {
    serial_rpc_write((uint8_t*)data, size);
  }
	}
}

void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 128;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}