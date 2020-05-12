/*
 * Copyright (c) 2019 ST Microelectronics Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_ipcc_mailbox

#include <drivers/clock_control.h>
#include <device.h>
#include <errno.h>
#include <drivers/ipm.h>
#include <soc.h>

#include <drivers/clock_control/stm32_clock_control.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ipm_stm32_ipcc, CONFIG_IPM_LOG_LEVEL);

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct stm32_ipcc_mailbox_config * const)(dev)->config_info)
#define DEV_DATA(dev)							\
	((struct stm32_ipcc_mbx_data * const)(dev)->driver_data)
#define MBX_STRUCT(dev)					\
	((IPCC_TypeDef *)(DEV_CFG(dev))->uconf.base)

#define IPCC_ALL_MR_TXF_CH_MASK 0xFFFF0000
#define IPCC_ALL_MR_RXO_CH_MASK 0x0000FFFF
#define IPCC_ALL_SR_CH_MASK	0x0000FFFF

#if (CONFIG_IPM_STM32_IPCC_PROCID == 1)

#define IPCC_EnableIT_TXF(hipcc)  LL_C1_IPCC_EnableIT_TXF(hipcc)
#define IPCC_DisableIT_TXF(hipcc)  LL_C1_IPCC_DisableIT_TXF(hipcc)

#define IPCC_EnableIT_RXO(hipcc)  LL_C1_IPCC_EnableIT_RXO(hipcc)
#define IPCC_DisableIT_RXO(hipcc)  LL_C1_IPCC_DisableIT_RXO(hipcc)

#define IPCC_EnableReceiveChannel(hipcc, ch)	\
			LL_C1_IPCC_EnableReceiveChannel(hipcc, 1 << ch)
#define IPCC_EnableTransmitChannel(hipcc, ch)	\
			LL_C1_IPCC_EnableTransmitChannel(hipcc, 1 << ch)
#define IPCC_DisableReceiveChannel(hipcc, ch)	\
			LL_C2_IPCC_DisableReceiveChannel(hipcc, 1 << ch)
#define IPCC_DisableTransmitChannel(hipcc, ch)	\
			LL_C1_IPCC_DisableTransmitChannel(hipcc, 1 << ch)

#define IPCC_ClearFlag_CHx(hipcc, ch)	LL_C1_IPCC_ClearFlag_CHx(hipcc, 1 << ch)
#define IPCC_SetFlag_CHx(hipcc, ch)	LL_C1_IPCC_SetFlag_CHx(hipcc, 1 << ch)

#define IPCC_IsActiveFlag_CHx(hipcc, ch)	\
			LL_C1_IPCC_IsActiveFlag_CHx(hipcc, 1 << ch)

#define IPCC_ReadReg(hipcc, reg) READ_REG(hipcc->C1##reg)

#define IPCC_ReadReg_SR(hipcc) READ_REG(hipcc->C1TOC2SR)
#define IPCC_ReadOtherInstReg_SR(hipcc) READ_REG(hipcc->C2TOC1SR)

#else

#define IPCC_EnableIT_TXF(hipcc)  LL_C2_IPCC_EnableIT_TXF(hipcc)
#define IPCC_DisableIT_TXF(hipcc)  LL_C2_IPCC_DisableIT_TXF(hipcc)

#define IPCC_EnableIT_RXO(hipcc)  LL_C2_IPCC_EnableIT_RXO(hipcc)
#define IPCC_DisableIT_RXO(hipcc)  LL_C2_IPCC_DisableIT_RXO(hipcc)

#define IPCC_EnableReceiveChannel(hipcc, ch)	\
			LL_C2_IPCC_EnableReceiveChannel(hipcc, 1 << ch)
#define IPCC_EnableTransmitChannel(hipcc, ch)	\
			LL_C2_IPCC_EnableTransmitChannel(hipcc, 1 << ch)
#define IPCC_DisableReceiveChannel(hipcc, ch)	\
			LL_C2_IPCC_DisableReceiveChannel(hipcc, 1 << ch)
#define IPCC_DisableTransmitChannel(hipcc, ch)	\
			LL_C2_IPCC_DisableTransmitChannel(hipcc, 1 << ch)

#define IPCC_ClearFlag_CHx(hipcc, ch)	LL_C2_IPCC_ClearFlag_CHx(hipcc, 1 << ch)
#define IPCC_SetFlag_CHx(hipcc, ch)	LL_C2_IPCC_SetFlag_CHx(hipcc, 1 << ch)

#define IPCC_IsActiveFlag_CHx(hipcc, ch)	\
			LL_C2_IPCC_IsActiveFlag_CHx(hipcc, 1 << ch)

#define IPCC_ReadReg(hipcc, reg) READ_REG(hipcc->C2##reg)

#define IPCC_ReadReg_SR(hipcc) READ_REG(hipcc->C2TOC1SR)
#define IPCC_ReadOtherInstReg_SR(hipcc) READ_REG(hipcc->C1TOC2SR)

#endif

struct stm32_ipcc_mailbox_config {
	void (*irq_config_func)(const struct device *dev);
	IPCC_TypeDef *ipcc;
	struct stm32_pclken pclken;
};

struct stm32_ipcc_mbx_data {
	u32_t num_ch;
	ipm_callback_t callback;
	void *callback_ctx;
};

static struct stm32_ipcc_mbx_data stm32_IPCC_data;

static void stm32_ipcc_mailbox_rx_isr(void *arg)
{
	const struct device *dev = arg;
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	const struct stm32_ipcc_mailbox_config *cfg = DEV_CFG(dev);
	unsigned int value = 0;
	u32_t mask, i;

	mask = (~IPCC_ReadReg(cfg->ipcc, MR)) & IPCC_ALL_MR_RXO_CH_MASK;
	mask &= IPCC_ReadOtherInstReg_SR(cfg->ipcc) & IPCC_ALL_SR_CH_MASK;

	for (i = 0; i < data->num_ch; i++) {
		if (!((1 << i) & mask)) {
			continue;
		}
		LOG_DBG("%s channel = %x\r\n", __func__, i);
		/* mask the channel Free interrupt  */
		IPCC_DisableReceiveChannel(cfg->ipcc, i);

		if (data->callback) {
			/* Only one MAILBOX, id is unused and set to 0 */
			data->callback(data->callback_ctx, i, &value);
		}
		/* clear status to acknoledge message reception */
		IPCC_ClearFlag_CHx(cfg->ipcc, i);
		IPCC_EnableReceiveChannel(cfg->ipcc, i);
	}
}

static void stm32_ipcc_mailbox_tx_isr(void *arg)
{
	const struct device *dev = arg;
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	const struct stm32_ipcc_mailbox_config *cfg = DEV_CFG(dev);
	u32_t mask, i;

	mask = (~IPCC_ReadReg(cfg->ipcc, MR)) & IPCC_ALL_MR_TXF_CH_MASK;
	mask = mask >> IPCC_C1MR_CH1FM_Pos;

	mask &= IPCC_ReadReg_SR(cfg->ipcc) & IPCC_ALL_SR_CH_MASK;

	for (i = 0; i <  data->num_ch; i++) {
		if (!((1 << i) & mask)) {
			continue;
		}
		LOG_DBG("%s channel = %x\r\n", __func__, i);
		/* mask the channel Free interrupt */
		IPCC_DisableTransmitChannel(cfg->ipcc, i);
	}
}

static int stm32_ipcc_mailbox_ipm_send(const struct device *dev, int wait,
				       u32_t id,
				       const void *buff, int size)
{
	struct stm32_ipcc_mbx_data *data = dev->driver_data;
	const struct stm32_ipcc_mailbox_config *cfg = DEV_CFG(dev);

	ARG_UNUSED(wait);
	ARG_UNUSED(buff);

	/* No data transmition, only doorbell */
	if (size) {
		return -EMSGSIZE;
	}

	if (id >= data->num_ch) {
		LOG_ERR("invalid id (%d)\r\n", id);
		return  -EINVAL;
	}

	LOG_DBG("Send msg on channel %d\r\n", id);

	/* Check that the channel is free (otherwise wait) */
	if (IPCC_IsActiveFlag_CHx(cfg->ipcc, id)) {
		LOG_DBG("Waiting for channel to be freed\r\n");
		while (IPCC_IsActiveFlag_CHx(cfg->ipcc, id)) {
			;
		}
	}
	IPCC_EnableTransmitChannel(cfg->ipcc, id);
	IPCC_SetFlag_CHx(cfg->ipcc, id);

	return 0;
}

static int stm32_ipcc_mailbox_ipm_max_data_size_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* no data transfer capability */
	return 0;
}

static u32_t stm32_ipcc_mailbox_ipm_max_id_val_get(const struct device *d)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(d);

	return data->num_ch - 1;
}

static void stm32_ipcc_mailbox_ipm_register_callback(const struct device *d,
						     ipm_callback_t cb,
						     void *context)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(d);

	data->callback = cb;
	data->callback_ctx = context;
}

static int stm32_ipcc_mailbox_ipm_set_enabled(const struct device *dev,
					      int enable)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	const struct stm32_ipcc_mailbox_config *cfg = DEV_CFG(dev);
	u32_t i;

	/* For now: nothing to be done */
	LOG_DBG("%s %s mailbox\r\n", __func__, enable ? "enable" : "disable");
	if (enable) {
		/* Enable RX and TX interrupts */
		IPCC_EnableIT_TXF(cfg->ipcc);
		IPCC_EnableIT_RXO(cfg->ipcc);
		for (i = 0; i < data->num_ch; i++) {
			IPCC_EnableReceiveChannel(cfg->ipcc, i);
		}
	} else {
		/* Disable RX and TX interrupts */
		IPCC_DisableIT_TXF(cfg->ipcc);
		IPCC_DisableIT_RXO(cfg->ipcc);
		for (i = 0; i < data->num_ch; i++) {
			IPCC_DisableReceiveChannel(cfg->ipcc, i);
		}
	}

	return 0;
}

static int stm32_ipcc_mailbox_init(const struct device *dev)
{

	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	const struct stm32_ipcc_mailbox_config *cfg = DEV_CFG(dev);
	const struct device *clk;
	u32_t i;

	clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
	__ASSERT_NO_MSG(clk);

	/* enable clock */
	if (clock_control_on(clk,
			     (clock_control_subsys_t *)&cfg->pclken) != 0) {
		return -EIO;
	}

	/* Disable RX and TX interrupts */
	IPCC_DisableIT_TXF(cfg->ipcc);
	IPCC_DisableIT_RXO(cfg->ipcc);

	data->num_ch = LL_IPCC_GetChannelConfig(cfg->ipcc);

	for (i = 0; i < data->num_ch; i++) {
		/* Clear RX status */
		IPCC_ClearFlag_CHx(cfg->ipcc, i);
		/* mask RX and TX interrupts */
		IPCC_DisableReceiveChannel(cfg->ipcc, i);
		IPCC_DisableTransmitChannel(cfg->ipcc, i);
	}

	cfg->irq_config_func(dev);

	return 0;
}

static const struct ipm_driver_api stm32_ipcc_mailbox_driver_api = {
	.send = stm32_ipcc_mailbox_ipm_send,
	.register_callback = stm32_ipcc_mailbox_ipm_register_callback,
	.max_data_size_get = stm32_ipcc_mailbox_ipm_max_data_size_get,
	.max_id_val_get = stm32_ipcc_mailbox_ipm_max_id_val_get,
	.set_enabled = stm32_ipcc_mailbox_ipm_set_enabled,
};

static void stm32_ipcc_mailbox_config_func(const struct device *dev);

/* Config MAILBOX 0 */
static const struct stm32_ipcc_mailbox_config stm32_ipcc_mailbox_0_config = {
	.irq_config_func = stm32_ipcc_mailbox_config_func,
	.ipcc = (IPCC_TypeDef *)DT_INST_REG_ADDR(0),
	.pclken = { .bus = DT_INST_CLOCKS_CELL(0, bus),
		    .enr = DT_INST_CLOCKS_CELL(0, bits)
	},

};

DEVICE_AND_API_INIT(mailbox_0, DT_INST_LABEL(0),
		    &stm32_ipcc_mailbox_init,
		    &stm32_IPCC_data, &stm32_ipcc_mailbox_0_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &stm32_ipcc_mailbox_driver_api);

static void stm32_ipcc_mailbox_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, rxo, irq),
		    DT_INST_IRQ_BY_NAME(0, rxo, priority),
		    stm32_ipcc_mailbox_rx_isr, DEVICE_GET(mailbox_0), 0);

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, txf, irq),
		    DT_INST_IRQ_BY_NAME(0, txf, priority),
		    stm32_ipcc_mailbox_tx_isr, DEVICE_GET(mailbox_0), 0);

	irq_enable(DT_INST_IRQ_BY_NAME(0, rxo, irq));
	irq_enable(DT_INST_IRQ_BY_NAME(0, txf, irq));
}
