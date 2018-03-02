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
#ifndef __PC_H__
#define __PC_H__

#include <stdbool.h>
#include <stdint.h>
#include <libopencm3/stm32/timer.h>

enum pc_channel {
	PC_CH1 = TIM_IC1,
	PC_CH2 = TIM_IC2,
};

struct period_counter_channel {
	bool active;

	uint32_t ovf;
	uint32_t cnt;
	uint32_t sem;

	uint32_t period;
	uint32_t total;
};

struct period_counter {
	uint32_t timer;
	bool active;

	struct period_counter_channel ch1;
	struct period_counter_channel ch2;
};

void period_counter_update(struct period_counter *pc);
void period_counter_init(struct period_counter *pc);
void period_counter_enable(struct period_counter *pc, enum pc_channel ch);
void period_counter_disable(struct period_counter *pc, enum pc_channel ch);
uint32_t period_counter_get(struct period_counter *pc, enum pc_channel ch);
uint32_t period_counter_get_total(struct period_counter *pc, enum pc_channel ch);
void period_counter_reset_total(struct period_counter *pc, enum pc_channel ch);

#endif /* __PC_H__ */
