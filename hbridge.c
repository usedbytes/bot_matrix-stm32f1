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
#include "hbridge.h"
#include "pwm.h"

static void channel_init_pwm(uint32_t timer, struct channel *c)
{
	pwm_channel_disable(timer, c->ch2);
	pwm_channel_set_duty(timer, c->ch2, 0);

	pwm_channel_disable(timer, c->ch1);
	pwm_channel_set_duty(timer, c->ch1, 0);

	c->dir = DIRECTION_NONE;
	c->duty = 0;
}

void hbridge_init(struct hbridge *hb)
{
	pwm_timer_init(hb->timer, 15000);
	pwm_timer_enable(hb->timer);

	channel_init_pwm(hb->timer, &hb->a);
	channel_init_pwm(hb->timer, &hb->b);
}

static void channel_refresh(uint32_t timer, struct channel *c)
{
	if (c->dir) {
		pwm_channel_set_duty(timer, c->ch2, c->duty);
	} else {
		pwm_channel_set_duty(timer, c->ch1, c->duty);
	}
}

void hbridge_set_freq(struct hbridge *hb, uint32_t frequency)
{
	pwm_timer_disable(hb->timer);
	pwm_timer_set_freq(hb->timer, frequency);

	channel_refresh(hb->timer, &hb->a);
	channel_refresh(hb->timer, &hb->b);

	pwm_timer_enable(hb->timer);
}

static void channel_set_direction(uint32_t timer, struct channel *c,
				  enum direction dir)
{
	if (dir == c->dir) {
		return;
	}

	if (dir) {
		pwm_channel_set_duty(timer, c->ch1, 0);
		pwm_channel_disable(timer, c->ch1);
		pwm_channel_set_duty(timer, c->ch2, 0);
		pwm_channel_enable(timer, c->ch2);
	} else {
		pwm_channel_set_duty(timer, c->ch2, 0);
		pwm_channel_disable(timer, c->ch2);
		pwm_channel_set_duty(timer, c->ch1, 0);
		pwm_channel_enable(timer, c->ch1);
	}

	c->dir = dir;
}

static void channel_set_duty(uint32_t timer, struct channel *c, uint16_t duty)
{
	if (duty > 64224) {
		duty = 64224; /* 98% max duty */
	}

	c->duty = duty;
	channel_refresh(timer, c);
}

void hbridge_set_duty(struct hbridge *hb, enum hbridge_channel chan,
		      enum direction dir, uint16_t duty)
{
	if (chan == HBRIDGE_A) {
		channel_set_direction(hb->timer, &hb->a, dir);
		channel_set_duty(hb->timer, &hb->a, duty);
	} else {
		channel_set_direction(hb->timer, &hb->b, dir);
		channel_set_duty(hb->timer, &hb->b, duty);
	}
}
