/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief USART driver for LPC54XXX family
 *
 * Note:
 * - Only basic USART features sufficient to support printf functionality
 *   are currently implemented.
 * - The driver works only in polling mode, interrupt mode is not implemented.
 */

#include <errno.h>
#include <device.h>
#include <uart.h>
#include <fsl_lpc_usart.h>
#include <fsl_clock.h>
#include <soc.h>
#include <fsl_device_registers.h>

struct usart_lpc_mcux_config {
	USART_Type *base;
	clock_name_t clock_source;
	u32_t baud_rate;
};

struct usart_lpc_mcux_data {
/*put irq call back instance when required*/
};

static int usart_lpc_mcux_poll_in(struct device *dev, unsigned char *c)
{
	const struct usart_lpc_mcux_config *config = dev->config->config_info;
	u32_t flags = USART_GetStatusFlags(config->base);
	int ret = -1;

	if (flags & kUSART_RxFifoFullFlag) {
		*c = USART_ReadByte(config->base);
		ret = 0;
	}

	return ret;
}

static unsigned char usart_lpc_mcux_poll_out(struct device *dev,
						unsigned char c)
{
	const struct usart_lpc_mcux_config *config = dev->config->config_info;

	/* Wait until space is available in TX FIFO */
	while (!(USART_GetStatusFlags(config->base) & kUSART_TxFifoEmptyFlag))
		;

	USART_WriteByte(config->base, c);

	/* Wait until TX is done transmitting */
	while (!(config->base->STAT & kStatus_USART_TxIdle))
		;

	return c;
}

static int usart_lpc_mcux_err_check(struct device *dev)
{
	const struct usart_lpc_mcux_config *config = dev->config->config_info;
	u32_t flags = USART_GetStatusFlags(config->base);
	int err = 0;

	if (flags & kStatus_USART_RxRingBufferOverrun) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & kStatus_USART_ParityError) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & kStatus_USART_FramingError) {
		err |= UART_ERROR_FRAMING;
	}

	USART_ClearStatusFlags(config->base, kStatus_USART_RxRingBufferOverrun |
		kStatus_USART_ParityError | kStatus_USART_FramingError);
	return err;
}

static int usart_lpc_mcux_init(struct device *dev)
{
	const struct usart_lpc_mcux_config *config = dev->config->config_info;
	usart_config_t usart_config;
	u32_t clock_freq;

	clock_freq = CLOCK_GetFreq(config->clock_source);

	USART_GetDefaultConfig(&usart_config);
	usart_config.enableTx = true;
	usart_config.enableRx = true;
	usart_config.baudRate_Bps = config->baud_rate;

	USART_Init(config->base, &usart_config, clock_freq);
	return 0;
}

static const struct uart_driver_api usart_lpc_mcux_driver_api = {
	.poll_in = usart_lpc_mcux_poll_in,
	.poll_out = usart_lpc_mcux_poll_out,
	.err_check = usart_lpc_mcux_err_check,
};

#ifdef CONFIG_USART_LPC_MCUX_0
static const struct usart_lpc_mcux_config usart_lpc_mcux_0_config = {
	.base = USART0,
	.clock_source = kCLOCK_Flexcomm0,
	.baud_rate = CONFIG_USART_LPC_MCUX_0_BAUD_RATE,
};

static struct usart_lpc_mcux_data usart_lpc_mcux_0_data;

DEVICE_AND_API_INIT(usart_0, CONFIG_USART_LPC_MCUX_0_NAME,
	&usart_lpc_mcux_init, &usart_lpc_mcux_0_data,
	&usart_lpc_mcux_0_config, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &usart_lpc_mcux_driver_api);

#endif /* CONFIG_USART_LPC_MCUX_0 */
