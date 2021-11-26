#include <inttypes.h>
#include <stdbool.h>

#ifndef PWM_H
#define PWM_H

struct __attribute__((packed, aligned(4))) pwmPacket {
	uint8_t enable: 1;
	uint8_t polarity: 1;
	uint32_t duty: 30;
	uint32_t period: 32;
};

struct __attribute__((packed, aligned(4))) pwmCapture {
	uint32_t duty: 32;
	uint32_t period: 32;
};

void pwm_init();

void captureFreq(uint8_t channel);

void configurePwm(uint8_t channel, bool enable, bool polarity, uint32_t duty_ns, uint32_t period_ns);

void pwm_capture_read_data();

#endif  //PWM_H