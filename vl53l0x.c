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
#include <errno.h>
#include "i2c.h"
#include "log.h"
#include "util.h"
#include "vl53l0x.h"

#include "vl53l0x/core/inc/vl53l0x_api.h"
#include "vl53l0x/core/inc/vl53l0x_def.h"
#include "vl53l0x/platform/inc/vl53l0x_platform.h"

static int vl53l0x_platform_errno;

#define to_dev(_Dev) ((struct vl53l0x_dev *)_Dev)

int vl53l0x_init(struct vl53l0x_dev *dev)
{
	VL53L0X_DEV pal_dev = &dev->pal_dev;
	VL53L0X_Error err = VL53L0X_ERROR_NONE;
	int ret;

	VL53L0X_DeviceInfo_t dev_info;
	uint32_t count;
	uint8_t vhv, phase, type;

	err = VL53L0X_DataInit(pal_dev);
	if (err) {
		log_err("DataInit: %x",(uint32_t)err);
		return err;
	}

	err = VL53L0X_GetDeviceInfo(pal_dev, &dev_info);
	if (err) {
		log_err("GetDeviceInfo: %x", (uint32_t)err);
		return err;
	}

	err = VL53L0X_StaticInit(pal_dev);
	if (err) {
		log_err("StaticInit: %x", (uint32_t)err);
		return err;
	}

	ret = vl53l0x_perform_ref_cal(dev, &vhv, &phase);
	if (ret) {
		return ret;
	}
	log_info("vhv: %d phase: %d\n", (uint32_t)vhv, (uint32_t)phase);

	ret = vl53l0x_perform_ref_spad(dev, &count, &type);
	if (ret) {
		return ret;
	}
	log_info("SPAD: %d ap: %d\n", count, (uint32_t)type);

	return 0;
}

int vl53l0x_perform_ref_cal(struct vl53l0x_dev *dev, uint8_t *vhv, uint8_t *phase)
{
	return VL53L0X_PerformRefCalibration(&dev->pal_dev, vhv, phase);
}

int vl53l0x_load_ref_cal(struct vl53l0x_dev *dev, uint8_t vhv, uint8_t phase)
{
	return VL53L0X_SetRefCalibration(&dev->pal_dev, vhv, phase);
}

int vl53l0x_perform_ref_spad(struct vl53l0x_dev *dev, uint32_t *count, uint8_t *type)
{
	return VL53L0X_PerformRefSpadManagement(&dev->pal_dev, count, type);
}

int vl53l0x_load_ref_spad(struct vl53l0x_dev *dev, uint32_t count, uint8_t type)
{
	return VL53L0X_SetReferenceSpads(&dev->pal_dev, count, type);
}

int vl53l0x_perform_offset_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, int32_t *offset_um)
{
	return VL53L0X_PerformOffsetCalibration(&dev->pal_dev, distance_mm, offset_um);
}

int vl53l0x_load_offset_cal(struct vl53l0x_dev *dev, int32_t offset_um)
{
	return VL53L0X_SetOffsetCalibrationDataMicroMeter(&dev->pal_dev, offset_um);
}

int vl53l0x_perform_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, FixPoint1616_t *xtalk)
{
	return VL53L0X_PerformXTalkCalibration(&dev->pal_dev, distance_mm, xtalk);
}

int vl53l0x_load_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t xtalk)
{
	return VL53L0X_SetXTalkCompensationRateMegaCps(&dev->pal_dev, xtalk);
}

int vl53l0x_enable_xtalk_compensation(struct vl53l0x_dev *dev)
{
	return VL53L0X_SetXTalkCompensationEnable(&dev->pal_dev, 1);
}

int vl53l0x_disable_xtalk_compensation(struct vl53l0x_dev *dev)
{
	return VL53L0X_SetXTalkCompensationEnable(&dev->pal_dev, 0);
}

int vl53l0x_set_measurement_time(struct vl53l0x_dev *dev, uint32_t us)
{
	return VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&dev->pal_dev, us);
}

int vl53l0x_set_measurement_mode(struct vl53l0x_dev *dev, VL53L0X_DeviceModes mode)
{
	return VL53L0X_SetDeviceMode(&dev->pal_dev, mode);
}

int vl53l0x_start_measurement(struct vl53l0x_dev *dev)
{
	return VL53L0X_StartMeasurement(&dev->pal_dev);
}

int vl53l0x_stop_measurement(struct vl53l0x_dev *dev)
{
	return VL53L0X_StopMeasurement(&dev->pal_dev);
}

int vl53l0x_check_stop_completed(struct vl53l0x_dev *dev)
{
	VL53L0X_Error err;
	uint32_t busy;

	err = VL53L0X_GetStopCompletedStatus(&dev->pal_dev, &busy);
	if (err) {
		return err;
	}

	if (!busy) {
		return 1;
	}

	return 0;
}

int vl53l0x_check_measurement_ready(struct vl53l0x_dev *dev)
{
	VL53L0X_Error err;
	uint8_t ready;

	err = VL53L0X_GetMeasurementDataReady(&dev->pal_dev, &ready);
	if (err) {
		return err;
	}

	if (ready) {
		return 1;
	}

	return 0;
}

int vl53l0x_get_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data)
{
	VL53L0X_Error err;

	err = VL53L0X_GetRangingMeasurementData(&dev->pal_dev, data);
	if (err) {
		return err;
	}

	/* TODO: Check error? */
	VL53L0X_ClearInterruptMask(&dev->pal_dev, 0);

	return 0;
}

int vl53l0x_do_single_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data)
{
	int i;
	int ret = vl53l0x_set_measurement_mode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (ret) {
		return ret;
	}

	ret = vl53l0x_start_measurement(dev);
	if (ret) {
		return ret;
	}

	for (i = 0; i < 50; i++) {
		ret = vl53l0x_check_measurement_ready(dev);
		if (ret < 0) {
			return ret;
		} else if (ret == 1) {
			break;
		}
		delay_ms(5);
	}

	if (i >= 50) {
		return -ETIMEDOUT;
	}

	return vl53l0x_get_measurement(dev, data);
}

int vl53l0x_get_platform_error(void)
{
	return vl53l0x_platform_errno;
}

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
		vl53l0x_platform_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	struct vl53l0x_dev *dev = to_dev(Dev);
	int ret = i2c_read(dev->addr_7b, index, pdata, count);
	if (ret) {
		vl53l0x_platform_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	struct vl53l0x_dev *dev = to_dev(Dev);
	int ret = i2c_write_byte(dev->addr_7b, index, data);
	if (ret) {
		vl53l0x_platform_errno = ret;
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
		vl53l0x_platform_errno = ret;
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
