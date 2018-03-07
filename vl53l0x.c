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
#include "i2c.h"
#include "util.h"

#include "vl53l0x/core/inc/vl53l0x_def.h"
#include "vl53l0x/platform/inc/vl53l0x_platform.h"

static int vl53l0x_errno;

/*
 * The STM API is a leaky as **** abstraction, it needs to have visibility
 * of the Dev structure because of its dumb Get/Set macros.
 * I still want a clean separation from the ST code, so do this subclassing
 * to avoid defining my internal structure in vl53l0x_platform.h
 */
#define to_dev(_Dev) ((struct vl53l0x_dev *)_Dev)
struct vl53l0x_dev {
	struct VL53L0X_Dev pal_dev;
	uint8_t addr_7b;
};

/******************************************************************************
 * STM API PAL Implementation
 ******************************************************************************/

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
	(void)Dev;
	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
	(void)Dev;
	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	struct vl53l0x_dev *dev = to_dev(Dev);
	int ret = i2c_write(dev->addr_7b, index, pdata, count);
	if (ret) {
		vl53l0x_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	struct vl53l0x_dev *dev = to_dev(Dev);
	int ret = i2c_read(dev->addr_7b, index, pdata, count);
	if (ret) {
		vl53l0x_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	struct vl53l0x_dev *dev = to_dev(Dev);
	int ret = i2c_write_byte(dev->addr_7b, index, data);
	if (ret) {
		vl53l0x_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	return VL53L0X_WriteMulti(Dev, index, (uint8_t *)&data, sizeof(data));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	return VL53L0X_WriteMulti(Dev, index, (uint8_t *)&data, sizeof(data));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	struct vl53l0x_dev *dev = to_dev(Dev);
	int ret = i2c_read_byte(dev->addr_7b, index, data);
	if (ret) {
		vl53l0x_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	return VL53L0X_ReadMulti(Dev, index, (uint8_t *)data, sizeof(*data));
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	return VL53L0X_ReadMulti(Dev, index, (uint8_t *)data, sizeof(*data));
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	uint8_t data;
	VL53L0X_Error err = VL53L0X_RdByte(Dev, index, &data);
	if (err != VL53L0X_ERROR_NONE) {
		return err;
	}

	data = (data & AndData) | OrData;

	return VL53L0X_WrByte(Dev, index, data);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	(void)Dev;
	delay_ms(5);
	return VL53L0X_ERROR_NONE;
}
