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

#ifndef __LOG_H__
#define __LOG_H__

#include <stdarg.h>

#define LOG_LEVEL_DEFAULT LOG_LEVEL_INFO

enum log_level {
	LOG_LEVEL_ERR = 0,
	LOG_LEVEL_WARN,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DBG,
};

/*
 *  __log_func should be defined to override the default null logger, with the
 *  following signature:
 */
void __log_func(enum log_level level, const char *str, unsigned int nargs, va_list args);

void log_printf(const char *str, ...);
void log_err(const char *str, ...);
void log_warn(const char *str, ...);
void log_info(const char *str, ...);
void log_dbg(const char *str, ...);

void log_vprintf(enum log_level level, const char *str, va_list args);

#endif /* __LOG_H__ */
