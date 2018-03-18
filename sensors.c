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
#include <errno.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>

#include "i2c.h"
#include "log.h"
#include "sensors.h"
#include "systick.h"
#include "util.h"
#include "vl53l0x.h"

enum report_type {
	REPORT_TYPE_RANGE = 0,
};

struct ranging_report {
	uint8_t sensor;
	uint8_t status;
	uint16_t range_mm;
	uint32_t signal;
	uint32_t ambient;
	uint16_t spad;
};

struct sensor_report {
	uint32_t timestamp;
	//enum report_type type;
	uint32_t type;
	union {
		/* REPORT_TYPE_RANGE */
		struct ranging_report range;
	};
};

enum request_type {
	REQUEST_TYPE_START = 0,
};

struct start_request {
	uint8_t sensor;
	uint8_t mode;
};

struct sensor_request {
	uint32_t request_id;
	//enum request_type type;
	uint32_t type;
	union {
		/* REQUEST_TYPE_START */
		struct start_request start;
	};
};

enum sensor_id {
	SENSOR_TOF_LEFT = 0,
	SENSOR_TOF_RIGHT,
};

struct vl53l0x_dev tofs[] = {
	[SENSOR_TOF_RIGHT] = {
		.addr_7b = 0x27,
		.id = SENSOR_TOF_RIGHT,
	},
	[SENSOR_TOF_LEFT] = {
		.addr_7b = 0x28,
		.id = SENSOR_TOF_LEFT,
		.xshut_port = GPIOB,
		.xshut_pin = GPIO0,
	},
};

void sensors_init(void)
{
	int i, ret;
	for (i = 0; i < 5; i++) {
		ret = vl53l0x_init_array(tofs, 2);
		if (!ret) {
			break;
		}
		if (ret < 0) {
			i2c_reset_bus();
		}
	}
	if (i == 5) {
		log_err("Unable to initialise sensors. %d", (uint32_t)ret);
	}
}

static int send_ranging_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data)
{
	struct sensor_report *report;
	struct spi_pl_packet *pkt = spi_alloc_packet();
	if (!pkt) {
		return -ENOMEM;
	}

	pkt->type = EP_SENSORS_REP;
	report = (struct sensor_report *)pkt->data;
	report->timestamp = msTicks;
	report->type = REPORT_TYPE_RANGE;
	report->range.sensor = dev->id;
	report->range.status = data->RangeStatus;
	report->range.range_mm = data->RangeMilliMeter;
	report->range.signal = data->SignalRateRtnMegaCps;
	report->range.ambient = data->AmbientRateRtnMegaCps;
	report->range.spad = data->EffectiveSpadRtnCount;

	spi_send_packet(pkt);
	return 0;
}

static int read_tof(struct vl53l0x_dev *dev)
{
	int ret;
	VL53L0X_RangingMeasurementData_t data;
	ret = vl53l0x_get_measurement(dev, &data);
	if (ret) {
		return ret;
	}

	return send_ranging_measurement(dev, &data);
}

void sensors_tick(void)
{
	unsigned int i;
	int ret;
	for (i = 0; i < ARRAY_SIZE(tofs); i++) {
		ret = vl53l0x_check_measurement_ready(&tofs[i]);
		//log_dbg("%d ready: %d", (uint32_t)i, (uint32_t)ret);
		if (ret == 1) {
			ret = read_tof(&tofs[i]);
			/* Fallthrough */
		}
		if (ret < 0 && ret != -EINVAL) {
			log_err("Sensor %d read error: %d", (uint32_t)i, (uint32_t)ret);
		}
	}
}

void sensors_handle_packet(struct spi_pl_packet *pkt)
{
	int ret;
	struct sensor_request *req = (struct sensor_request *)pkt->data;
	//log_dbg("Handling sensor packet");
	switch (req->type) {
		case (REQUEST_TYPE_START):
		{
			struct vl53l0x_dev *dev = &tofs[req->start.sensor];
			vl53l0x_set_measurement_mode(dev, req->start.mode);

			ret = vl53l0x_set_measurement_time(dev, 40000);
			if (ret) {
				log_err("MeasureTime: %x", (uint32_t)ret);
				return;
			}

			ret = vl53l0x_start_measurement(dev);
			if (ret) {
				log_err("Sensor %d start error: %d", (uint32_t)req->start.sensor, (uint32_t)ret);
			} else {
				//log_dbg("Started... %d %d", (uint32_t)req->start.sensor, (uint32_t)req->start.mode);
			}
		}
	}
}
