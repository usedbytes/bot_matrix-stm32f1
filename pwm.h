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
#include <stdint.h>

void pwm_timer_init(uint32_t timer_peripheral, uint32_t frequency);
void pwm_timer_enable(uint32_t timer_peripheral);
void pwm_timer_disable(uint32_t timer_peripheral);

void pwm_channel_enable(uint32_t timer_peripheral, uint32_t channel);
void pwm_channel_disable(uint32_t timer_peripheral, uint32_t channel);
void pwm_channel_set_duty(uint32_t timer_peripheral, uint32_t channel,
			  uint16_t duty);
