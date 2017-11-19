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
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <stdint.h>

#define FP_VAL(_x) ((uint32_t)(_x * 65536))

struct gain {
	int32_t Kc;
	int32_t Kd;
	int32_t Ki;
};

struct controller {
	struct gain gain;
	bool gain_override;
	const struct gain *gains;
	unsigned int ngains;
	int32_t ilimit;

	uint32_t set_point;
	int32_t ierr;
	int32_t err;
	int skip;

	uint32_t (*process)(void *);
	void *closure;
};

void controller_init(struct controller *c, const struct gain *gains, uint8_t ngains);
void controller_set_gains(struct controller *c, int32_t Kc, int32_t Kd, int32_t Ki);
void controller_set_ilimit(struct controller *c, int32_t ilimit);
void controller_set(struct controller *c, uint32_t set_point);
uint32_t controller_get(struct controller *c);
int32_t controller_tick(struct controller *c, uint32_t pv, uint8_t gs_idx);

#endif /* __CONTROLLER_H__ */
