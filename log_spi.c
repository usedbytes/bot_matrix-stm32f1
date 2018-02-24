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
#include <stdint.h>
#include <string.h>

#include "log.h"
#include "spi.h"

#define MESSAGE_PKT_TYPE 0xff
struct message_pkt {
	uint8_t level;
	uint8_t n_args;
	uint8_t pad[2];
	// uint32_t args[]
	// char str[]
};

void __log_func(enum log_level level, const char *str, unsigned int n_args, va_list args)
{
	int ret;
	unsigned int i;
	struct message_pkt *pp;

	struct spi_pl_packet *pkt = spi_alloc_packet();
	if (!pkt) {
		return;
	}

	pkt->type = MESSAGE_PKT_TYPE;
	pp = (struct message_pkt *)pkt->data;
	pp->level = level;
	pp->n_args = n_args;

	/* First offset */
	ret = sizeof(*pp);

	for (i = 0; i < n_args; i++) {
		uint32_t arg = va_arg(args, uint32_t);

		ret = spi_packetise_stream(pkt, ret, (char *)&arg, sizeof(arg));
		if (ret < 0) {
			goto fail;
		}
	}

	ret = spi_packetise_stream(pkt, ret, str, strlen(str) + 1);
	if (ret < 0) {
		goto fail;
	}

	spi_send_packet(pkt);
	return;

fail:
	spi_free_packet(pkt);
}
