#include <stdbool.h>

#include "vl53l0x/core/inc/vl53l0x_def.h"
#include "vl53l0x/platform/inc/vl53l0x_platform.h"

struct vl53l0x_dev {
	struct VL53L0X_Dev pal_dev;
	uint8_t addr_7b;

	uint32_t xshut_port;
	uint16_t xshut_pin;

	bool addr_set : 1;
};

int vl53l0x_init(struct vl53l0x_dev *dev);
int vl53l0x_init_array(struct vl53l0x_dev *devs, unsigned ndevs);
int vl53l0x_set_addr(struct vl53l0x_dev *dev, uint8_t new_addr_7b);
int vl53l0x_get_platform_error(void);

int vl53l0x_perform_ref_cal(struct vl53l0x_dev *dev, uint8_t *vhv, uint8_t *phase);
int vl53l0x_load_ref_cal(struct vl53l0x_dev *dev, uint8_t vhv, uint8_t phase);

int vl53l0x_perform_ref_spad(struct vl53l0x_dev *dev, uint32_t *count, uint8_t *type);
int vl53l0x_load_ref_spad(struct vl53l0x_dev *dev, uint32_t count, uint8_t type);

int vl53l0x_perform_offset_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, int32_t *offset_um);
int vl53l0x_load_offset_cal(struct vl53l0x_dev *dev, int32_t offset_um);

int vl53l0x_perform_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, FixPoint1616_t *xtalk);
int vl53l0x_load_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t xtalk);
int vl53l0x_enable_xtalk_compensation(struct vl53l0x_dev *dev);
int vl53l0x_disable_xtalk_compensation(struct vl53l0x_dev *dev);

int vl53l0x_set_measurement_time(struct vl53l0x_dev *dev, uint32_t us);

int vl53l0x_set_measurement_mode(struct vl53l0x_dev *dev, VL53L0X_DeviceModes mode);
int vl53l0x_start_measurement(struct vl53l0x_dev *dev);
int vl53l0x_stop_measurement(struct vl53l0x_dev *dev);
int vl53l0x_check_stop_completed(struct vl53l0x_dev *dev);
int vl53l0x_check_measurement_ready(struct vl53l0x_dev *dev);
int vl53l0x_get_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data);

int vl53l0x_do_single_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data);
