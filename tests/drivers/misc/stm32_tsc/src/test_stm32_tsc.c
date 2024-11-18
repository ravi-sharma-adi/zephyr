/*
 * Copyright (c) 2024 Arif Balik <arifbalik@outlook.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Verify STM32 TSC peripheral is configured properly and can be started
 *
 * @details
 * - Test Steps
 *   -# Get a TSC device
 *   -# Verify the device is ready
 *   -# Verify MIMO region with device tree values
 *   -# Test the acquisition in polling mode
 *   -# Test the acquisition in interrupt mode
 * - Expected Results
 *   -# The device is ready
 *   -# The device tree values are correctly mapped to the TSC registers
 *   -# The acquisition is successful in polling mode
 *   -# The acquisition is successful in interrupt mode
 *
 */

#include <autoconf.h>
#include <inttypes.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/misc/stm32_tsc/stm32_tsc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_tsc)
#define TSC_NODE DT_INST(0, st_stm32_tsc)
#else
#error "Define a TSC device"
#endif

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct gpio_dt_spec signal_mock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, signal_gpios);

const struct device *tsc_device_get(void)
{
	return DEVICE_DT_GET(TSC_NODE);
}

const struct gpio_dt_spec *tsc_signal_mock_get(void)
{
	return &signal_mock;
}

ZTEST(stm32_tsc, test_1_device_ready)
{
	const struct device *dev = tsc_device_get();

	zassert_true(device_is_ready(dev), "STM32 TSC device is not ready");
}

ZTEST(stm32_tsc, test_2_cr_reg)
{
	TSC_TypeDef *tsc = (TSC_TypeDef *)DT_REG_ADDR(TSC_NODE);

	volatile const uint32_t *tsc_cr = &tsc->CR;
	const uint8_t pgpsc = DT_PROP(TSC_NODE, pulse_generator_prescaler);
	const uint8_t ctph = DT_PROP(TSC_NODE, ctph);
	const uint8_t ctpl = DT_PROP(TSC_NODE, ctpl);
	const uint8_t ssd = DT_PROP_OR(TSC_NODE, spread_spectrum_deviation, 0);
	const bool spread_spectrum = DT_PROP_OR(TSC_NODE, spread_spectrum, false);
	const uint8_t sscpsc = DT_PROP_OR(TSC_NODE, spread_spectrum_prescaler, 0);
	const uint8_t max_count = DT_PROP_OR(TSC_NODE, max_count_value, 0);
	const bool iodef = DT_PROP_OR(TSC_NODE, iodef_float, false);
	const bool sync_pol = DT_PROP_OR(TSC_NODE, syncpol_rising, false);
	const bool sync_acq = DT_PROP_OR(TSC_NODE, synced_acquisition, false);

	/* check charge transfer pulse high value (bits 31:28) */
	zassert_equal((*tsc_cr & TSC_CR_CTPL_Msk) >> TSC_CR_CTPL_Pos, ctph - 1,
		      "CTPH value is not correct, expected %d, got %d", ctph - 1,
		      (*tsc_cr & TSC_CR_CTPL_Msk) >> TSC_CR_CTPL_Pos);

	/* check charge transfer pulse low value (bits 27:24) */
	zassert_equal((*tsc_cr & TSC_CR_CTPL_Msk) >> TSC_CR_CTPL_Pos, ctpl - 1,
		      "CTPL value is not correct, expected %d, got %d", ctpl - 1,
		      (*tsc_cr & TSC_CR_CTPL_Msk) >> TSC_CR_CTPL_Pos);

	/* check spread spectrum deviation value (bits 23:17) */
	zassert_equal((*tsc_cr & TSC_CR_SSD_Msk) >> TSC_CR_SSD_Pos, ssd,
		      "SSD value is not correct, expected %d, got %d", ssd,
		      (*tsc_cr & TSC_CR_SSD_Msk) >> TSC_CR_SSD_Pos);

	/* check spread spectrum enable bit (bit 16) */
	if (spread_spectrum) {
		zexpect_true(*tsc_cr & TSC_CR_SSE_Msk);
	} else {
		zexpect_false(*tsc_cr & TSC_CR_SSE_Msk);
	}

	/* check spread spectrum prescaler value (bits 15) */
	if (sscpsc == 2) {
		zexpect_true(*tsc_cr & TSC_CR_SSPSC_Msk);
	} else {
		zexpect_false(*tsc_cr & TSC_CR_SSPSC_Msk);
	}

	/* check pulse generator prescaler value (bits 14:12) */
	zassert_equal((*tsc_cr & TSC_CR_PGPSC_Msk) >> TSC_CR_PGPSC_Pos, pgpsc,
		      "PGPSC value is not correct, expected %d, got %d", pgpsc,
		      (*tsc_cr & TSC_CR_PGPSC_Msk) >> TSC_CR_PGPSC_Pos);

	/* check max count value (bits 7:5) */
	zassert_equal((*tsc_cr & TSC_CR_MCV_Msk) >> TSC_CR_MCV_Pos, max_count,
		      "MCV value is not correct, expected %d, got %d", max_count,
		      (*tsc_cr & TSC_CR_MCV_Msk) >> TSC_CR_MCV_Pos);

	/* check I/O default mode bit (bit 4) */
	if (iodef) {
		zexpect_true(*tsc_cr & TSC_CR_IODEF_Msk);
	} else {
		zexpect_false(*tsc_cr & TSC_CR_IODEF_Msk);
	}

	/* check sync polarity bit (bit 3) */
	if (sync_pol) {
		zexpect_true(*tsc_cr & TSC_CR_SYNCPOL_Msk);
	} else {
		zexpect_false(*tsc_cr & TSC_CR_SYNCPOL_Msk);
	}

	/* check sync acquisition bit (bit 2) */
	if (sync_acq) {
		zexpect_true(*tsc_cr & TSC_CR_AM_Msk);
	} else {
		zexpect_false(*tsc_cr & TSC_CR_AM_Msk);
	}

	/* check start bit (bit 1) */
	zexpect_false(*tsc_cr & TSC_CR_START_Msk);

	/* check TSC enable bit (bit 0) */
	zexpect_true(*tsc_cr & TSC_CR_TSCE_Msk);
}

ZTEST(stm32_tsc, test_3_group_registers)
{
	TSC_TypeDef *tsc = (TSC_TypeDef *)DT_REG_ADDR(TSC_NODE);
	volatile const uint32_t *tsc_iohcr = &tsc->IOHCR;
	volatile const uint32_t *tsc_ioscr = &tsc->IOSCR;
	volatile const uint32_t *tsc_ioccr = &tsc->IOCCR;
	volatile const uint32_t *tsc_iogcsr = &tsc->IOGCSR;

#define GET_GROUP_BITS(val) (uint32_t)(((val) & 0x0f) << ((group - 1) * 4))

#define STM32_TSC_GROUP_TEST(node)                                                                 \
	do {                                                                                       \
		const uint8_t group = DT_PROP(node, group);                                        \
		const uint8_t channel_ios = DT_PROP(node, channel_ios);                            \
		const uint8_t sampling_io = DT_PROP(node, sampling_io);                            \
		const bool use_as_shield = DT_PROP(node, use_as_shield);                           \
                                                                                                   \
		/* check schmitt trigger hysteresis for enabled I/Os */                            \
		zassert_equal((*tsc_iohcr & GET_GROUP_BITS(channel_ios | sampling_io)), 0,         \
			      "Schmitt trigger hysteresis not disabled, expected %d, got %d", 0,   \
			      (*tsc_iohcr & GET_GROUP_BITS(channel_ios | sampling_io)));           \
                                                                                                   \
		/* check channel I/Os */                                                           \
		zassert_equal(                                                                     \
			(*tsc_ioccr & GET_GROUP_BITS(channel_ios)), GET_GROUP_BITS(channel_ios),   \
			"Channel I/Os value is not correct, expected %d, got %d",                  \
			GET_GROUP_BITS(channel_ios), (*tsc_ioccr & GET_GROUP_BITS(channel_ios)));  \
                                                                                                   \
		/* check sampling I/O */                                                           \
		zassert_equal(                                                                     \
			(*tsc_ioscr & GET_GROUP_BITS(sampling_io)), GET_GROUP_BITS(sampling_io),   \
			"Sampling I/O value is not correct, expected %d, got %d",                  \
			GET_GROUP_BITS(sampling_io), (*tsc_ioscr & GET_GROUP_BITS(sampling_io)));  \
                                                                                                   \
		/* check enabled groups */                                                         \
		if (use_as_shield)                                                                 \
			zassert_not_equal((*tsc_iogcsr & BIT(group - 1)), BIT(group - 1),          \
					  "Group %d is a shield group and should not be enabled",  \
					  group);                                                  \
		else                                                                               \
			zassert_equal((*tsc_iogcsr & BIT(group - 1)), BIT(group - 1),              \
				      "Group %d is not enabled", group);                           \
	} while (0)

#define GROUP_TEST_RUN(node) STM32_TSC_GROUP_TEST(node);

	DT_FOREACH_CHILD_STATUS_OKAY(TSC_NODE, GROUP_TEST_RUN);
}

ZTEST(stm32_tsc, test_4_acquisition_polling)
{
	Z_TEST_SKIP_IFDEF(CONFIG_STM32_TSC_INTERRUPT);

	const struct device *tsc_dev = tsc_device_get();
	TSC_TypeDef *tsc = (TSC_TypeDef *)DT_REG_ADDR(TSC_NODE);
	volatile const uint32_t *tsc_cr = &tsc->CR;
	volatile const uint32_t *tsc_ier = &tsc->IER;
	volatile const uint32_t *tsc_isr = &tsc->ISR;

	stm32_tsc_start(tsc_dev);

	/* this should timeout because sync pin is not triggered */
	zexpect_not_ok(stm32_tsc_poll_for_acquisition(tsc_dev, K_MSEC(100)));

	stm32_tsc_start(tsc_dev);

	zexpect_ok(gpio_pin_toggle_dt(&signal_mock));

	/* check if interrupts are disabled */
	zexpect_false(*tsc_ier & TSC_IER_EOAIE_Msk);
	zexpect_false(*tsc_ier & TSC_IER_MCEIE_Msk);

	/* test CR register start bit */
	zexpect_true((*tsc_cr & TSC_CR_START_Msk) >> TSC_CR_START_Pos);

	zexpect_ok(stm32_tsc_poll_for_acquisition(tsc_dev, K_MSEC(100)));

	/* test for max count error flag */
	zexpect_false((*tsc_isr & TSC_ISR_MCEF_Msk) >> TSC_ISR_MCEF_Pos);

	uint32_t value;

	zexpect_ok(stm32_tsc_read(tsc_dev, 6, &value));
}

static volatile bool tsc_input_received;

static void tsc_input_callback(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(evt);
	ARG_UNUSED(user_data);

	tsc_input_received = true;
}
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(TSC_NODE), tsc_input_callback, NULL);

ZTEST(stm32_tsc, test_5_acquisition_interrupt)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_STM32_TSC_INTERRUPT);

	TSC_TypeDef *tsc = (TSC_TypeDef *)DT_REG_ADDR(TSC_NODE);
	volatile const uint32_t *tsc_cr = &tsc->CR;
	volatile const uint32_t *tsc_ier = &tsc->IER;
	volatile const uint32_t *tsc_isr = &tsc->ISR;

	tsc_input_received = false;

	stm32_tsc_start(tsc_device_get());

	/* test CR register start bit */
	zexpect_true((*tsc_cr & TSC_CR_START_Msk) >> TSC_CR_START_Pos);

	/* check if interrupts are enabled */
	zexpect_true(*tsc_ier & TSC_IER_EOAIE_Msk);
	zexpect_true(*tsc_ier & TSC_IER_MCEIE_Msk);

	k_sleep(K_MSEC(100));

	/* test ISR register max count error flag */
	zexpect_false((*tsc_isr & TSC_ISR_MCEF_Msk) >> TSC_ISR_MCEF_Pos);

	/* this should fail because of the sync pin */
	zexpect_false(tsc_input_received);

	zexpect_ok(gpio_pin_toggle_dt(&signal_mock));

	k_sleep(K_MSEC(100));

	zexpect_true(tsc_input_received);
}
