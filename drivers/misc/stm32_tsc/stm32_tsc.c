/*
 * Copyright (c) 2024 Arif Balik <arifbalik@outlook.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_tsc

#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

LOG_MODULE_REGISTER(stm32_tsc, CONFIG_LOG_DEFAULT_LEVEL);

struct stm32_tsc_group_config {
	const uint8_t group;
	const uint8_t channel_ios;
	const uint8_t sampling_io;
	const bool use_as_shield;
	const uint32_t zephyr_code;
};

struct stm32_tsc_config {
	TSC_TypeDef *tsc;
	const struct stm32_pclken *pclken;
	const struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	const struct stm32_tsc_group_config *groups;
	const uint8_t group_cnt;

	const uint8_t pgpsc;
	const uint8_t ctph;
	const uint8_t ctpl;
	const bool spread_spectrum;
	const uint8_t sscpsc;
	const uint8_t ssd;
	const uint16_t max_count;
	const bool iodef;
	const bool sync_acq;
	const bool sync_pol;
	void (*irq_func)(void);
};

void stm32_tsc_start(const struct device *dev)
{
	const struct stm32_tsc_config *config = dev->config;

	/* clear interrupts */
	sys_set_bits((mem_addr_t)&config->tsc->ICR, TSC_ICR_EOAIC | TSC_ICR_MCEIC);

#if IS_ENABLED(CONFIG_STM32_TSC_INTERRUPT)
	/* enable end of acquisition and max count error interrupts */
	sys_set_bits((mem_addr_t)&config->tsc->IER, TSC_IER_EOAIE | TSC_IER_MCEIE);
#endif

	/* TODO: When sync acqusition mode is enabled, both this bit and an external input signal
	 * should be set. When the acqusition stops this bit is cleared, so even if a sync signal is
	 * present, the next acqusition will not start until this bit is set again.
	 */
	/* start acquisition */
	sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_START_Pos);
}

int stm32_tsc_poll_for_acquisition(const struct device *dev, k_timeout_t timeout)
{
	const struct stm32_tsc_config *config = dev->config;
	uint32_t status;
	uint32_t start_tick = sys_clock_tick_get_32();

	do {
		status = sys_read32((mem_addr_t)&config->tsc->ISR);

		if (sys_clock_tick_get_32() - start_tick > timeout.ticks) {
			LOG_ERR("TSC@%p: timeout", config->tsc);
			return -ETIMEDOUT;
		}

	} while (!(status & TSC_ISR_EOAF) && !(status & TSC_ISR_MCEF));

	if (status & TSC_ISR_MCEF) {
		sys_set_bit((mem_addr_t)&config->tsc->ICR, TSC_ICR_MCEIC_Pos);
		LOG_ERR("TSC@%p: max count error", config->tsc);
		return -EIO;
	}

	sys_set_bit((mem_addr_t)&config->tsc->ICR, TSC_ICR_EOAIC_Pos);

	return 0;
}

int stm32_tsc_read(const struct device *dev, uint8_t group, uint32_t *value)
{
	const struct stm32_tsc_config *config = dev->config;

	/* make sure group starts with 1 and can go max to IOGXCR count */
	if (value == NULL || group < 1 || group > 8) {
		return -EINVAL;
	}

	*value = sys_read32((mem_addr_t)&config->tsc->IOGXCR[group - 1]);

	return 0;
}

static int stm32_tsc_handle_incoming_data(const struct device *dev)
{
	const struct stm32_tsc_config *config = dev->config;

	if (sys_test_bit((mem_addr_t)&config->tsc->ISR, TSC_ISR_MCEF_Pos)) {
		/* clear max count error flag */
		sys_set_bit((mem_addr_t)&config->tsc->ICR, TSC_ICR_MCEIC_Pos);
		LOG_ERR("TSC@%p: max count error", config->tsc);
		LOG_HEXDUMP_DBG(config->tsc, sizeof(TSC_TypeDef), "TSC Registers");
		return -EIO;
	}

	if (sys_test_bit((mem_addr_t)&config->tsc->ISR, TSC_ISR_EOAF_Pos)) {
		/* clear end of acquisition flag */
		sys_set_bit((mem_addr_t)&config->tsc->ICR, TSC_ICR_EOAIC_Pos);

		/* read values */
		for (int i = 0; i < config->group_cnt; i++) {
			const struct stm32_tsc_group_config *group = &config->groups[i];
			uint32_t group_bit = BIT(group->group - 1) << 16;

			if (config->tsc->IOGCSR & group_bit) {
				int32_t count_value = sys_read32(
					(mem_addr_t)&config->tsc->IOGXCR[group->group - 1]);
				input_report(dev, INPUT_EV_DEVICE, group->zephyr_code, count_value,
					     true, K_FOREVER);
			}
		}
	}

	return 0;
}

static void stm32_tsc_isr(const struct device *dev)
{
	const struct stm32_tsc_config *config = dev->config;

	/* disable interrupts */
	sys_clear_bits((mem_addr_t)&config->tsc->IER, TSC_IER_EOAIE | TSC_IER_MCEIE);

	stm32_tsc_handle_incoming_data(dev);
}

static int stm32_tsc_init(const struct device *dev)
{
	const struct stm32_tsc_config *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	int ret;

	if (!device_is_ready(clk)) {
		LOG_ERR("TSC@%p: clock controller device not ready", config->tsc);
		return -ENODEV;
	}

	/* reset TSC values to default */
	ret = reset_line_toggle_dt(&config->reset);
	if (ret < 0) {
		LOG_ERR("Failed to reset TSC@%p (%d)", config->tsc, ret);
		return ret;
	}

	ret = clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]);
	if (ret < 0) {
		LOG_ERR("Failed to enable clock for TSC@%p (%d)", config->tsc, ret);
		return ret;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure TSC@%p pins (%d)", config->tsc, ret);
		return ret;
	}

	/* set ctph (bits 31:28) and ctpl (bits 27:24) */
	sys_set_bits((mem_addr_t)&config->tsc->CR, (((config->ctph - 1) << 4) | (config->ctpl - 1))
							   << TSC_CR_CTPL_Pos);

	/* set spread spectrum deviation (bits 23:17) */
	sys_set_bits((mem_addr_t)&config->tsc->CR, config->ssd << TSC_CR_SSD_Pos);

	/* set pulse generator prescaler (bits 14:12) */
	sys_set_bits((mem_addr_t)&config->tsc->CR, config->pgpsc << TSC_CR_PGPSC_Pos);

	/* set max count value (bits 7:5) */
	sys_set_bits((mem_addr_t)&config->tsc->CR, config->max_count << TSC_CR_MCV_Pos);

	/* set spread spectrum prescaler (bit 15) */
	if (config->sscpsc == 2) {
		sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_SSPSC_Pos);
	}

	/* set sync bit polarity */
	if (config->sync_pol) {
		sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_SYNCPOL_Pos);
	}

	/* set sync acquisition */
	if (config->sync_acq) {
		sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_AM_Pos);
	}

	/* set I/O default mode */
	if (config->iodef) {
		sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_IODEF_Pos);
	}

	/* set spread spectrum */
	if (config->spread_spectrum) {
		sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_SSE_Pos);
	}

	/* group configuration */
	for (int i = 0; i < config->group_cnt; i++) {
		const struct stm32_tsc_group_config *group = &config->groups[i];

		if (group->channel_ios & group->sampling_io) {
			LOG_ERR("TSC@%p: group %d has the same channel and sampling I/O",
				config->tsc, group->group);
			return -EINVAL;
		}

		/* if use_as_shield is true, the channel I/Os are used as shield, and can only have
		 * values 1,2,4,8
		 */
		if (group->use_as_shield && group->channel_ios != 1 && group->channel_ios != 2 &&
		    group->channel_ios != 4 && group->channel_ios != 8) {
			LOG_ERR("TSC@%p: group %d is used as shield, but has invalid channel I/Os. "
				"Can only have one",
				config->tsc, group->group);
			return -EINVAL;
		}

		/* each group only has 4 configurable I/O */
#define GET_GROUP_BITS(val) (uint32_t)(((val) & 0x0f) << ((group->group - 1) * 4))

		/* clear schmitt trigger hysteresis for enabled I/Os */
		sys_clear_bits((mem_addr_t)&config->tsc->IOHCR,
			       GET_GROUP_BITS(group->channel_ios | group->sampling_io));

		/* set channel I/Os */
		sys_set_bits((mem_addr_t)&config->tsc->IOCCR, GET_GROUP_BITS(group->channel_ios));

		/* set sampling I/O */
		sys_set_bits((mem_addr_t)&config->tsc->IOSCR, GET_GROUP_BITS(group->sampling_io));

		/* enable group */
		if (!group->use_as_shield) {
			sys_set_bit((mem_addr_t)&config->tsc->IOGCSR, group->group - 1);
		}
	}

	/* disable interrupts */
	sys_clear_bits((mem_addr_t)&config->tsc->IER, TSC_IER_EOAIE | TSC_IER_MCEIE);

	/* clear interrupts */
	sys_set_bits((mem_addr_t)&config->tsc->ICR, TSC_ICR_EOAIC | TSC_ICR_MCEIC);

	/* enable peripheral */
	sys_set_bit((mem_addr_t)&config->tsc->CR, TSC_CR_TSCE_Pos);

#if IS_ENABLED(CONFIG_STM32_TSC_INTERRUPT)
	config->irq_func();
#endif

	return 0;
}

#define STM32_TSC_GROUP_AND_COMMA(node_id)                                                         \
	{                                                                                          \
		.group = DT_PROP(node_id, group),                                                  \
		.channel_ios = DT_PROP(node_id, channel_ios),                                      \
		.sampling_io = DT_PROP(node_id, sampling_io),                                      \
		.use_as_shield = DT_PROP(node_id, use_as_shield),                                  \
		.zephyr_code = DT_PROP(node_id, zephyr_code),                                      \
	},

#define STM32_TSC_INIT(index)                                                                      \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);           \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static void stm32_tsc_irq_init_##index(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), stm32_tsc_isr,      \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	};                                                                                         \
                                                                                                   \
	static const struct stm32_tsc_group_config groups_cfg_##index[] = {                        \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(index, STM32_TSC_GROUP_AND_COMMA)};              \
                                                                                                   \
	static const struct stm32_tsc_config stm32_tsc_cfg_##index = {                             \
		.tsc = (TSC_TypeDef *)DT_INST_REG_ADDR(index),                                     \
		.pclken = pclken_##index,                                                          \
		.reset = RESET_DT_SPEC_INST_GET_OR(index, NULL),                                   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.groups = groups_cfg_##index,                                                      \
		.group_cnt = DT_INST_CHILD_NUM_STATUS_OKAY(index),                                 \
		.pgpsc = DT_INST_PROP_OR(index, pulse_generator_prescaler, 2),                     \
		.ctph = DT_INST_PROP_OR(index, ctph, 2),                                           \
		.ctpl = DT_INST_PROP_OR(index, ctpl, 2),                                           \
		.spread_spectrum = DT_INST_PROP_OR(index, spread_spectrum, false),                 \
		.sscpsc = DT_INST_PROP_OR(index, spread_spectrum_prescaler, 1),                    \
		.ssd = DT_INST_PROP_OR(index, spread_spectrum_deviation, 1),                       \
		.max_count = DT_INST_PROP_OR(index, max_count_value, 5),                           \
		.iodef = DT_INST_PROP_OR(index, iodef_float, false),                               \
		.sync_acq = DT_INST_PROP_OR(index, synced_acquisition, false),                     \
		.sync_pol = DT_INST_PROP_OR(index, syncpol_rising, false),                         \
		.irq_func = stm32_tsc_irq_init_##index,                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(index, stm32_tsc_init, NULL, NULL, &stm32_tsc_cfg_##index,           \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(STM32_TSC_INIT)
