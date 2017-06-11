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
#include <libopencm3/stm32/timer.h>

#include "counter.h"

void counter_timer_init(uint32_t timer_peripheral)
{
	timer_reset(timer_peripheral);

	timer_slave_set_mode(timer_peripheral, TIM_SMCR_SMS_ECM1);
	timer_slave_set_trigger(timer_peripheral, TIM_SMCR_TS_TI1FP1);
	timer_slave_set_polarity(timer_peripheral, TIM_ET_RISING);
}

void counter_enable(uint32_t timer_peripheral)
{
	timer_enable_counter(timer_peripheral);
}

void counter_disable(uint32_t timer_peripheral)
{
	timer_disable_counter(timer_peripheral);
}

void counter_reset(uint32_t timer_peripheral)
{
	TIM_CNT(timer_peripheral) = 0;
}

uint16_t counter_get_value(uint32_t timer_peripheral)
{
	return TIM_CNT(timer_peripheral);
}
