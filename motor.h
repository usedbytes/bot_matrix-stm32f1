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

#ifndef __MOTOR_H__
#define __MOTOR_H__
#include <stdint.h>

#include "spi.h"
#include "hbridge.h"

void motor_init(void);
void motor_disable_loop(void);
void motor_enable_loop(void);
void motor_process_packet(struct spi_pl_packet *pkt);
void motor_set_speed(enum hbridge_channel channel, enum direction dir,
		     uint16_t speed);
#endif /* __MOTOR_H__ */
