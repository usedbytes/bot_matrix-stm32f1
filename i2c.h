/*
 * Copyright (C) 2018 Brian Starkey <stark3y@gmail.com>
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
#ifndef __I2C_H__
#define __I2C_H__
#include <stdint.h>

void i2c_init(void);
void i2c_reset_bus(void);
/* Returns 1 if present, 0 if not, -1 on error */
int i2c_detect(uint8_t addr_7b);

int i2c_write(uint8_t addr_7b, uint8_t reg, uint8_t *data, unsigned int len);
int i2c_read(uint8_t addr_7b, uint8_t reg, uint8_t *data, unsigned int len);

int i2c_write_byte(uint8_t addr_7b, uint8_t reg, uint8_t data);
int i2c_read_byte(uint8_t addr_7b, uint8_t reg, uint8_t *data);

#endif /* __I2C_H__ */
