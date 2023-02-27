/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/gpio.h>

#include "coremark_zephyr.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/*
 * Get button and led configuration from the devicetree.
 * Application expects to have button and led aliases present in devicetree.
 */
#define BUTTON_NODE             DT_ALIAS(button)
#define LED_NODE                DT_ALIAS(led)

#if !DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
	#error "Unsupported board: /"button/" alias is not defined."
#endif

#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
	#error "Unsupported board: /"led/" alias is not defined."
#endif

#define BUTTON_LABEL            DT_PROP(DT_ALIAS(button), label)

static K_SEM_DEFINE(start_coremark, 0, 1);
static bool coremark_in_progress;

static const struct gpio_dt_spec start_button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static struct gpio_callback button_cb_data;

static void flush_log(void)
{
	if (IS_ENABLED(CONFIG_LOG_PROCESS_THREAD)) {
		while (log_data_pending()) {
			k_sleep(K_MSEC(10));
		}
		k_sleep(K_MSEC(10));
	} else {
		while (LOG_PROCESS()) {
		}
	}
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&start_button) && !coremark_in_progress) {
		LOG_INF("%s pressed!", BUTTON_LABEL);
		coremark_in_progress = true;
		k_sem_give(&start_coremark);
	}
}

static void led_init(void)
{
	int ret;

	if (!device_is_ready(status_led.port)) {
		LOG_INF("Error: led device %s is not ready",
			status_led.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		LOG_INF("Error %d: failed to configure %s pin %d",
			ret, status_led.port->name, status_led.pin);
		return;
	}

	gpio_pin_set_dt(&status_led, GPIO_ACTIVE_HIGH);
}

static void button_init(void)
{
	int ret;

	if (!device_is_ready(start_button.port)) {
		LOG_INF("Error: button1 device %s is not ready",
			start_button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&start_button, GPIO_INPUT);
	if (ret != 0) {
		LOG_INF("Error %d: failed to configure %s pin %d",
			ret, start_button.port->name, start_button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&start_button, GPIO_INT_EDGE_BOTH);

	if (ret != 0) {
		LOG_INF("Error %d: failed to configure interrupt on %s pin %d",
			ret, start_button.port->name, start_button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(start_button.pin));
	gpio_add_callback(start_button.port, &button_cb_data);
}

int main(void)
{
	LOG_INF("Coremark sample for %s", CONFIG_BOARD);

	led_init();
	button_init();

	if (IS_ENABLED(CONFIG_APP_MODE_FLASH_AND_RUN)) {
		coremark_in_progress = true;
		k_sem_give(&start_coremark);
	} else {
		LOG_INF("Press %s to start the test ...",  BUTTON_LABEL);
	}

	while (true) {
		k_sem_take(&start_coremark, K_FOREVER);
		LOG_INF("Coremark started!");
		LOG_INF("CPU FREQ: %d Hz", SystemCoreClock);
		LOG_INF("(threads: %d, data size: %d; iterations: %d)\n",
			CONFIG_COREMARK_THREADS_NUMBER,
			CONFIG_COREMARK_DATA_SIZE,
			CONFIG_COREMARK_ITERATIONS);
		flush_log();

		gpio_pin_set_dt(&status_led, GPIO_ACTIVE_LOW);
		coremark_run();
		gpio_pin_set_dt(&status_led, GPIO_ACTIVE_HIGH);

		LOG_INF("Coremark finished! Press %s to restart ...\n", BUTTON_LABEL);

		coremark_in_progress = false;
	};

	return 0;
}
