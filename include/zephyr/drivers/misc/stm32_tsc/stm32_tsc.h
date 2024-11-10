/*
 * Copyright (c) 2024 Arif Balik <arifbalik@outlook.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_STM32_TSC_H_
#define ZEPHYR_DRIVERS_MISC_STM32_TSC_H_

#include <zephyr/sys_clock.h>
#include <zephyr/types.h>

void stm32_tsc_start(const struct device *dev);
int stm32_tsc_poll_for_acquisition(const struct device *dev, k_timeout_t timeout);
int stm32_tsc_read(const struct device *dev, uint8_t group, uint32_t *value);

#endif /* ZEPHYR_DRIVERS_MISC_STM32_TSC_H_ */
