/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief New thread creation for ARM64 Cortex-A
 *
 * Core thread related primitives for the ARM64 Cortex-A
 */

#include <kernel.h>
#include <ksched.h>
#include <wait_q.h>
#include <arch/cpu.h>

void z_thread_entry_wrapper(k_thread_entry_t k, void *p1, void *p2, void *p3);

/*
 * init_stack_frame:
 *
 * SP
 * ^
 * | +------------------------------+
 * | | entry_point + arg1/arg2/arg3 | to setup z_thread_entry()
 * | +------------------------------+
 * | | SPSR + ELR                   | used by eret to jump back from SVC
 * | +------------------------------+
 * | | callee-saved (only x30 used) | popped-out by z_arm64_context_switch()
 * + +------------------------------+
 */
struct init_stack_frame {
	/* The only callee-saved register we are interested in is x30 */
	_callee_saved_stack_t callee;

	/* SPSL_ELn and ELR_ELn */
	uint64_t spsr;
	uint64_t elr;

	/*
	 * Used by z_thread_entry_wrapper. pulls these off the stack and
	 * into argument registers before calling z_thread_entry()
	 */
	uint64_t entry_point;
	uint64_t arg1;
	uint64_t arg2;
	uint64_t arg3;

	/* least recently pushed */
};

/*
 * An initial context, to be "restored" by z_arm64_context_switch(), is put at
 * the other end of the stack, and thus reusable by the stack when not needed
 * anymore.
 */
void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     char *stack_ptr, k_thread_entry_t entry,
		     void *p1, void *p2, void *p3)
{
	struct init_stack_frame *pInitCtx;

	pInitCtx = Z_STACK_PTR_TO_FRAME(struct init_stack_frame, stack_ptr);

	pInitCtx->entry_point = (uint64_t)entry;
	pInitCtx->arg1 = (uint64_t)p1;
	pInitCtx->arg2 = (uint64_t)p2;
	pInitCtx->arg3 = (uint64_t)p3;

	/*
	 * - ELR_ELn: to be used by eret in z_thread_entry_wrapper() to return
	 *   to z_thread_entry() with entry in x0(entry_point) and the
	 *   parameters already in place in x1(arg1), x2(arg2), x3(arg3).
	 * - SPSR_ELn: to enable IRQs (we are masking debug exceptions, SError
	 *   interrupts and FIQs).
	 */
	pInitCtx->elr = (uint64_t)z_thread_entry;
	pInitCtx->spsr = SPSR_MODE_EL1H | DAIF_FIQ;

	/*
	 * The only callee-register we need is x30 to be used by ret in
	 * z_arm64_context_switch() when the new task is first scheduled.
	 */
	pInitCtx->callee.x30 = (uint64_t)z_thread_entry_wrapper;

	/*
	 * We are saving SP to pop out:
	 * - x30 to jump from z_arm64_context_switch() to
	 *   z_thread_entry_wrapper()
	 * - SPSR and ELR to be restored by eret in z_thread_entry_wrapper()
	 * - entry_point and arguments when going through
	 *   z_thread_entry_wrapper().
	 */
	thread->callee_saved.sp = (uint64_t)pInitCtx;
}
