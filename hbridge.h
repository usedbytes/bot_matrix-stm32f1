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

#ifndef __HBRIDGE_H__
#define __HBRIDGE_H__

#include <stdint.h>
#include <stdbool.h>

enum hbridge_channel {
	HBRIDGE_A = 0,
	HBRIDGE_B,
};

enum direction {
	DIRECTION_FWD = 0,
	DIRECTION_REV = 0,
};

struct channel {
	/* Initisalise these */
	uint32_t ch1, ch2;

	/* These will be updated dynamically */
	enum direction dir;
	uint16_t duty;
};

struct hbridge {
	uint32_t timer;
	uint32_t counter;
	struct channel a;
	struct channel b;
};

void hbridge_init(struct hbridge *hb);

void hbridge_set_freq(struct hbridge *hb, uint32_t freq);

void hbridge_set_duty(struct hbridge *hb, enum hbridge_channel chan,
		      enum direction dir, uint16_t duty);
#endif /* __HBRIDGE__ */
