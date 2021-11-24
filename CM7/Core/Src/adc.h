#include <inttypes.h>

#ifndef ADC_H
#define ADC_H

enum AnalogPins {
	A0 = 0x1,
	A1,
	A2,
	A3,
	A4,
	A5,
	A6,
	A7,
};

uint16_t get_ADC_value(enum AnalogPins name);

void adc_init();

#endif  //ADC_H