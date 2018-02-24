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

#include "log.h"

void __attribute__((weak)) __log_func(enum log_level level, const char *str, unsigned int nargs, va_list args)
{
	(void)(level);
	(void)(str);
	(void)(nargs);
	(void)(args);

	/* Do nothing */
}

void log_vprintf(enum log_level level, const char *str, va_list args)
{
	const char *p = str;
	int nargs = 0;

	while (*p) {
		if (p[0] == '%') {
			if (p[1] == '%') {
				p = p + 2;
				continue;
			}
			nargs++;
		}
		p++;
	}

	__log_func(level, str, nargs, args);
}

void log_err(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	log_vprintf(LOG_LEVEL_ERR, str, args);
	va_end(args);
}

void log_warn(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	log_vprintf(LOG_LEVEL_WARN, str, args);
	va_end(args);
}

void log_printf(const char *str, ...) {
	va_list args;
	va_start(args, str);
	log_vprintf(LOG_LEVEL_DEFAULT, str, args);
	va_end(args);
}

void log_info(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	log_vprintf(LOG_LEVEL_INFO, str, args);
	va_end(args);
}

void log_dbg(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	log_vprintf(LOG_LEVEL_DBG, str, args);
	va_end(args);
}
