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
#include <stdio.h>
#include <stdbool.h>

#include "controller.h"

char dbg[256];
volatile bool cp;

void controller_init(struct controller *c, uint32_t (*process)(void *), void *d) {
	c->process = process;
	c->closure = d;
	c->skip = 1;
}

void controller_set_gains(struct controller *c, int32_t Kc, int32_t Kd, int32_t Ki) {
	c->Kc = Kc;
	c->Kd = Kd;
	c->Ki = Ki;
}

void controller_set(struct controller *c, uint32_t set_point) {
	c->set_point = set_point;
}

void controller_set_ilimit(struct controller *c, int32_t ilimit) {
	c->ilimit = ilimit;
}

int32_t controller_tick(struct controller *c) {
	int32_t tc, td, ti, ret;
	int32_t err, derr;
	uint32_t pv = c->process(c->closure);
	if (pv == 0) {
		c->skip++;
		return 0;
	}

	err = c->set_point - pv;
	c->ierr += ((err * 25) / (256 * c->skip));
	if (c->ierr > c->ilimit) {
		c->ierr = c->ilimit;
	} else if (c->ierr < -c->ilimit) {
		c->ierr = -c->ilimit;
	}
	derr = (err - c->err) * (10 * c->skip);
	c->err = err;
	c->skip = 1;

	tc = ((int64_t)(err) * c->Kc) / 65536;
	td = ((int64_t)(derr) * c->Kd) / 65536;
	ti = ((int64_t)(c->ierr) * c->Ki) / 65536;
	ret = tc + td + ti;

	sprintf(dbg, "err: %li, derr: %li, ierr: %li\r\n"
		"tc: %li, td: %li, ti: %li ret: %li\r\n", err, derr, c->ierr, tc, td, ti, ret);
	cp = true;

	return ret;
}
