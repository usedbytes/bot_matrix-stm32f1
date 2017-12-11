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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#include "period_counter.h"

void period_counter_update(struct period_counter *pc)
{
	if (timer_get_flag(pc->timer, TIM_SR_UIF)) {
		timer_clear_flag(pc->timer, TIM_SR_UIF);

		if (pc->ch1.active)
			pc->ch1.ovf++;
		if (pc->ch2.active)
			pc->ch2.ovf++;
	}

	if (timer_get_flag(pc->timer, TIM_SR_CC1IF)) {
		struct period_counter_channel *c = &pc->ch1;
		uint16_t cc = TIM_CCR1(TIM4);
		c->period = ((c->ovf << 16) + cc) - c->cnt;
		c->ovf = 0;
		c->cnt = cc;
		c->sem = true;

		timer_clear_flag(pc->timer, TIM_SR_CC1IF);
	}

	if (timer_get_flag(pc->timer, TIM_SR_CC2IF)) {
		struct period_counter_channel *c = &pc->ch2;
		uint16_t cc = TIM_CCR2(TIM4);
		c->period = ((c->ovf << 16) + cc) - c->cnt;
		c->ovf = 0;
		c->cnt = cc;
		c->sem = true;

		timer_clear_flag(pc->timer, TIM_SR_CC2IF);
	}
}

void period_counter_init(struct period_counter *pc)
{
	uint32_t timer = pc->timer;

	memset(pc, 0, sizeof(*pc));
	pc->timer = timer;

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      GPIO_TIM4_CH1 | GPIO_TIM4_CH2);
	timer_reset(timer);
	timer_slave_set_mode(timer, TIM_SMCR_SMS_OFF);
	timer_set_prescaler(timer, 710);
	timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_preload(timer);
	timer_update_on_overflow(timer);
	timer_enable_update_event(timer);
	timer_generate_event(timer, TIM_EGR_UG);

	timer_ic_set_input(timer, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(timer, TIM_IC2, TIM_IC_IN_TI2);

	timer_enable_irq(timer, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM4_IRQ);
}

void period_counter_enable(struct period_counter *pc, enum pc_channel ch)
{
	timer_ic_enable(pc->timer, ch);

	switch (ch) {
	case PC_CH1:
		// TODO: reset channel...
		pc->ch1.active = true;
		timer_enable_irq(pc->timer, TIM_DIER_CC1IE);
		break;
	case PC_CH2:
		// TODO: reset channel...
		pc->ch2.active = true;
		timer_enable_irq(pc->timer, TIM_DIER_CC2IE);
		break;
	}

	if (!pc->active) {
		// TODO: reset counter...
		timer_enable_counter(pc->timer);
		pc->active = true;
	}
}

void period_counter_disable(struct period_counter *pc, enum pc_channel ch)
{
	timer_ic_disable(pc->timer, ch);

	switch (ch) {
	case PC_CH1:
		timer_disable_irq(pc->timer, TIM_DIER_CC1IE);
		pc->ch1.active = false;
		break;
	case PC_CH2:
		timer_disable_irq(pc->timer, TIM_DIER_CC2IE);
		pc->ch2.active = false;
		break;
	}

	if (!pc->ch1.active && !pc->ch2.active) {
		timer_disable_counter(pc->timer);
		pc->active = false;
	}
}

uint32_t period_counter_get(struct period_counter *pc, enum pc_channel ch)
{
	struct period_counter_channel *c;
	switch (ch) {
	case PC_CH1:
		c = &pc->ch1;
		break;
	case PC_CH2:
		c = &pc->ch2;
		break;
	default:
		c = NULL;
	}

	if (c->active && c->sem) {
		return c->period;
	}

	return 0;
}
