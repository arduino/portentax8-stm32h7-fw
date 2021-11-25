#include "peripherals.h"

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