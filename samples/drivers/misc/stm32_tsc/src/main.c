/*
 * Copyright (c) 2024 Arif Balik <arifbalik@outlook.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <autoconf.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/misc/stm32_tsc/stm32_tsc.h>

/* Input subsystem only implemented for interrupts. */
static void tsc_input_callback(struct input_event *evt, void *user_data)
{
	printf("TSC input event: dev %s, type %u, sync %u, code %u, value %d\n", evt->dev->name,
	       evt->type, evt->sync, evt->code, evt->value);
}
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(tsc)), tsc_input_callback, NULL);

int main(void)
{
	const struct device *tsc_dev = DEVICE_DT_GET(DT_NODELABEL(tsc));

	if (!device_is_ready(tsc_dev)) {
		printf("TSC device is not ready\n");
		return -ENODEV;
	}

	while (1) {
		/* This will start the acqusition and print the readings to input subsystem */
		stm32_tsc_start(tsc_dev);

		/* You can alternatively disable interrupts and
		 * poll for acquisition completion.
		 */
#if !IS_ENABLED(CONFIG_STM32_TSC_INTERRUPT)
		int ret = stm32_tsc_poll_for_acquisition(tsc_dev, K_MSEC(100));

		if (ret < 0) {
			printf("Failed to poll for acquisition\n");
			return ret;
		}

		uint32_t value;

		ret = stm32_tsc_read(tsc_dev, 6, &value);
		if (ret < 0) {
			printf("Failed to read TSC\n");
			return ret;
		}

		printf("TSC value: %u\n", value);
#endif

		k_sleep(K_MSEC(100));
	}

	return 0;
}
