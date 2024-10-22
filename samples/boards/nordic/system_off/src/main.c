/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "retained.h"

#include <inttypes.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_gpio.h>
#if CONFIG_SOC_NRF54H20_CPUAPP
#include <hal/nrf_memconf.h>
#endif

static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec sw1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const uint32_t port_sw1 = DT_PROP(DT_GPIO_CTLR_BY_IDX(DT_ALIAS(sw1), gpios, 0), port);

int main(void)
{
	int rc;
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t nrf_pin_sw1 = 32 * port_sw1 + sw1.pin;
	bool do_poweroff = true;

	if (!device_is_ready(cons)) {
		printf("%s: device not ready.\n", cons->name);
		return 0;
	}

	if (nrf_gpio_pin_latch_get(nrf_pin_sw1)) {
		nrf_gpio_pin_latch_clear(nrf_pin_sw1);
		do_poweroff = false;
	}

	printf("\n%s system off demo\n", CONFIG_BOARD);

	if (IS_ENABLED(CONFIG_APP_USE_NRF_RETENTION) || IS_ENABLED(CONFIG_APP_USE_RETAINED_MEM)) {
		bool retained_ok = retained_validate();

		/* Increment for this boot attempt and update. */
		retained.boots += 1;
		retained_update();

		printf("Retained data: %s\n", retained_ok ? "valid" : "INVALID");
		printf("Boot count: %u\n", retained.boots);
		printf("Off count: %u\n", retained.off_count);
		printf("Active Ticks: %" PRIu64 "\n", retained.uptime_sum);
	} else {
		printf("Retained data not supported\n");
	}

	/* configure sw0 as input, interrupt as level active to allow wake-up */
	rc = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
	if (rc < 0) {
		printf("Could not configure sw0 GPIO (%d)\n", rc);
		return 0;
	}

	rc = gpio_pin_configure_dt(&sw1, GPIO_INPUT);
	if (rc < 0) {
		printf("Could not configure sw1 GPIO (%d)\n", rc);
		return 0;
	}

	rc = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_LEVEL_ACTIVE);
	if (rc < 0) {
		printf("Could not configure sw0 GPIO interrupt (%d)\n", rc);
		return 0;
	}

	rc = gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_LEVEL_ACTIVE);
	if (rc < 0) {
		printf("Could not configure sw0 GPIO interrupt (%d)\n", rc);
		return 0;
	}

	if (do_poweroff) {
		printf("Entering system off; press sw0 or sw1 to restart\n");
	} else {
		printf("Button sw1 pressed, not entering system off\n");
	}

	rc = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	if (rc < 0) {
		printf("Could not suspend console (%d)\n", rc);
		return 0;
	}

	if (IS_ENABLED(CONFIG_APP_USE_NRF_RETENTION) || IS_ENABLED(CONFIG_APP_USE_RETAINED_MEM)) {
		/* Update the retained state */
		retained.off_count += 1;
		retained_update();
	}

	if (do_poweroff) {
#if CONFIG_SOC_NRF54H20_CPUAPP
		/* Local RAM0 (TCM) is currently not used so retention can be disabled. */
		nrf_memconf_ramblock_ret_mask_enable_set(NRF_MEMCONF, 0, RAMBLOCK_RET_MASK, false);
		nrf_memconf_ramblock_ret_mask_enable_set(NRF_MEMCONF, 1, RAMBLOCK_RET_MASK, false);
#endif
		sys_poweroff();
	} else {
		k_sleep(K_FOREVER);
	}

	return 0;
}
