/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cache.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

extern void sys_arch_reboot(int type);

FUNC_NORETURN void sys_reboot(int type)
{
	(void)irq_lock();

	/* Disable caches to ensure all data is flushed */
	sys_cache_data_disable();
	sys_cache_instr_disable();

	if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
		sys_clock_disable();
	}

	sys_arch_reboot(type);

	/* should never get here */
	printk("Failed to reboot: spinning endlessly...\n");
	for (;;) {
		k_cpu_idle();
	}
}
