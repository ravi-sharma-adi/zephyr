/* ST Microelectronics LIS2DUXS12 smart accelerometer APIs
 *
 * Copyright (c) 2024 STMicroelectronics
 * Copyright (c) 2023 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stmemsc.h>

#include "lis2duxs12_reg.h"
#include <zephyr/dt-bindings/sensor/lis2dux12.h>

#include <zephyr/drivers/sensor.h>

#ifndef ZEPHYR_DRIVERS_SENSOR_LIS2DUXS12_ST_LIS2DUXS12_H_
#define ZEPHYR_DRIVERS_SENSOR_LIS2DUXS12_ST_LIS2DUXS12_H_

extern const struct lis2dux12_chip_api st_lis2duxs12_chip_api;

int st_lis2duxs12_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_LIS2DUXS12_ST_LIS2DUXS12_H_ */
