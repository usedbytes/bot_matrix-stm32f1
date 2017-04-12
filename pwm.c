/*
 * Copyright (C) 2017 Brian Starkey <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <stdint.h>
#include <stdio.h>

#include "pwm.h"

void pwm_timer_init(uint32_t timer_peripheral, uint32_t frequency) {
	uint32_t pre = 1;
	uint32_t div, period = 0;

	/* Reset everything */
	timer_reset(timer_peripheral);
	timer_slave_set_mode(timer_peripheral, TIM_SMCR_SMS_OFF);

	/* Set prescaler */
	div = 36000000 / frequency;
	while (!period) {
		if (div > 65535) {
			pre++;
			div = 36000000 / (frequency * pre);
		} else {
			period = div;
		}
	}
	timer_set_prescaler(timer_peripheral, pre);
	timer_set_period(timer_peripheral, period);

	/* Set mode */
	timer_set_mode(timer_peripheral, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_preload(timer_peripheral);
	timer_update_on_overflow(timer_peripheral);
	timer_enable_update_event(timer_peripheral);
	timer_generate_event(timer_peripheral, TIM_EGR_UG);

	/* Set all channels as PWM */
	timer_set_oc_mode(timer_peripheral, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(timer_peripheral, TIM_OC1);
	timer_enable_oc_preload(timer_peripheral, TIM_OC1);

	timer_set_oc_mode(timer_peripheral, TIM_OC2, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(timer_peripheral, TIM_OC2);
	timer_enable_oc_preload(timer_peripheral, TIM_OC2);

	timer_set_oc_mode(timer_peripheral, TIM_OC3, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(timer_peripheral, TIM_OC3);
	timer_enable_oc_preload(timer_peripheral, TIM_OC3);

	timer_set_oc_mode(timer_peripheral, TIM_OC4, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(timer_peripheral, TIM_OC4);
	timer_enable_oc_preload(timer_peripheral, TIM_OC4);

	/* Setup AFIO */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      GPIO_TIM2_CH1_ETR | GPIO_TIM2_CH2 |
		      GPIO_TIM2_CH3 | GPIO_TIM2_CH4);
}

void pwm_timer_enable(uint32_t timer_peripheral) {
	timer_enable_counter(timer_peripheral);
}

void pwm_timer_disable(uint32_t timer_peripheral) {
	timer_disable_counter(timer_peripheral);
}

void pwm_channel_enable(uint32_t timer_peripheral, uint32_t channel) {
	timer_enable_oc_output(timer_peripheral, channel);
}

void pwm_channel_disable(uint32_t timer_peripheral, uint32_t channel) {
	timer_disable_oc_output(timer_peripheral, channel);
}

void pwm_channel_set_duty(uint32_t timer_peripheral, uint32_t channel,
			  uint16_t duty) {
	uint32_t period = TIM_ARR(timer_peripheral);
	duty = (period * duty) >> 16;
	printf("Period: %lu duty %u\r\n", period, duty);

	timer_set_oc_value(timer_peripheral, channel, duty);
}
