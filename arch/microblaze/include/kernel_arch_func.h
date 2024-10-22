/*
 * Copyright (c) 2023 Advanced Micro Devices, Inc. (AMD)
 * Copyright (c) 2023 Alp Sayin <alpsayin@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/**
 * @file
 * @brief Private kernel definitions
 *
 * This file contains private kernel function/macro definitions and various
 * other definitions for the MIPS processor architecture.
 */

#ifndef ZEPHYR_ARCH_MICROBLAZE_INCLUDE_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_MICROBLAZE_INCLUDE_KERNEL_ARCH_FUNC_H_

#include <kernel_arch_data.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE
static ALWAYS_INLINE void arch_kernel_init(void)
{
}

static ALWAYS_INLINE void arch_thread_return_value_set(struct k_thread *thread, unsigned int value)
{
	thread->callee_saved.retval = value;
}

FUNC_NORETURN void z_microblaze_fatal_error(unsigned int reason, const struct arch_esf *esf);

static inline bool arch_is_in_isr(void)
{
	return _kernel.cpus[0].nested != 0U;
}

#ifdef CONFIG_IRQ_OFFLOAD
void z_irq_do_offload(void);
#endif

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_ARCH_MICROBLAZE_INCLUDE_KERNEL_ARCH_FUNC_H_ */
