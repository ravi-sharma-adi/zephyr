/*
 * Copyright (c) 2024 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef REG_H_
#define REG_H_

#define reg_read(reg)                                                                              \
	({                                                                                         \
		register unsigned long __rv;                                                       \
		__asm__ volatile("mv %0, " STRINGIFY(reg) : "=r"(__rv));                           \
		__rv;                                                                              \
	})

#define reg_write(reg, val) ({ __asm__("mv " STRINGIFY(reg) ", %0" : : "r"(val)); })

#endif /* REG_H_ */
