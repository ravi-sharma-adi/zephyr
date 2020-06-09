/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hoperf_th02

#include <kernel.h>
#include <device.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "th02.h"

LOG_MODULE_REGISTER(TH02, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t read8(struct device *dev, uint8_t d)
{
	uint8_t buf;

	if (i2c_reg_read_byte(dev, TH02_I2C_DEV_ID, d, &buf) < 0) {
		LOG_ERR("Error reading register.");
	}
	return buf;
}

static int is_ready(struct device *dev)
{

	uint8_t status;

	if (i2c_reg_read_byte(dev, TH02_I2C_DEV_ID,
			      TH02_REG_STATUS, &status) < 0) {
		LOG_ERR("error reading status register");
	}

	if (status & TH02_STATUS_RDY_MASK) {
		return 0;
	} else {
		return 1;
	}
}

static uint16_t get_humi(struct device *dev)
{
	uint16_t humidity = 0U;

	if (i2c_reg_write_byte(dev, TH02_I2C_DEV_ID,
			       TH02_REG_CONFIG, TH02_CMD_MEASURE_HUMI) < 0) {
		LOG_ERR("Error writing register");
		return 0;
	}
	while (!is_ready(dev)) {
	}

	humidity = read8(dev, TH02_REG_DATA_H) << 8;
	humidity |= read8(dev, TH02_REG_DATA_L);
	humidity >>= 4;

	return humidity;
}

uint16_t get_temp(struct device *dev)
{
	uint16_t temperature = 0U;

	if (i2c_reg_write_byte(dev, TH02_I2C_DEV_ID,
			       TH02_REG_CONFIG, TH02_CMD_MEASURE_TEMP) < 0) {
		LOG_ERR("Error writing register");
		return 0;
	}
	while (!is_ready(dev)) {
	}

	temperature = read8(dev, TH02_REG_DATA_H) << 8;
	temperature |= read8(dev, TH02_REG_DATA_L);
	temperature >>= 2;

	return temperature;
}

static int th02_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct th02_data *drv_data = dev->driver_data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	drv_data->t_sample = get_temp(drv_data->i2c);
	LOG_INF("temp: %u", drv_data->t_sample);
	drv_data->rh_sample = get_humi(drv_data->i2c);
	LOG_INF("rh: %u", drv_data->rh_sample);

	return 0;
}

static int th02_channel_get(struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	struct th02_data *drv_data = dev->driver_data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP ||
			chan == SENSOR_CHAN_HUMIDITY);

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		/* val = sample / 32 - 50 */
		val->val1 = drv_data->t_sample / 32U - 50;
		val->val2 = (drv_data->t_sample % 32) * (1000000 / 32);
	} else {
		/* val = sample / 16 -24 */
		val->val1 = drv_data->rh_sample / 16U - 24;
		val->val2 = (drv_data->rh_sample % 16) * (1000000 / 16);
	}

	return 0;
}

static const struct sensor_driver_api th02_driver_api = {
	.sample_fetch = th02_sample_fetch,
	.channel_get = th02_channel_get,
};

static int th02_init(struct device *dev)
{
	struct th02_data *drv_data = dev->driver_data;

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			    DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	return 0;
}

static struct th02_data th02_driver;

DEVICE_DEFINE(th02, DT_INST_LABEL(0), th02_init, device_pm_control_nop,
		    &th02_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &th02_driver_api);
