/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_flexcan

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <fsl_flexcan.h>
#include <zephyr/irq.h>
#include "can_mcux_flexcan_common.h"

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

LOG_MODULE_DECLARE(can_mcux_flexcan, CONFIG_CAN_LOG_LEVEL);

#define MCUX_FLEXCAN_MAX_TX \
	(FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(0) \
	- MCUX_FLEXCAN_MAX_RX)

struct mcux_flexcan_rx_callback {
	flexcan_rx_mb_config_t mb_config;
	flexcan_frame_t frame;
	can_rx_callback_t function;
	void *arg;
};

struct mcux_flexcan_tx_callback {
	flexcan_frame_t frame;
	can_tx_callback_t function;
	void *arg;
};

struct mcux_flexcan_classic_data {
	const struct device *dev;
	flexcan_handle_t handle;

	ATOMIC_DEFINE(rx_allocs, MCUX_FLEXCAN_MAX_RX);
	struct k_mutex rx_mutex;
	struct mcux_flexcan_rx_callback rx_cbs[MCUX_FLEXCAN_MAX_RX];

	ATOMIC_DEFINE(tx_allocs, MCUX_FLEXCAN_MAX_TX);
	struct k_sem tx_allocs_sem;
	struct k_mutex tx_mutex;
	struct mcux_flexcan_tx_callback tx_cbs[MCUX_FLEXCAN_MAX_TX];
	enum can_state state;
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	struct can_timing timing;
	bool started;
};

static int mcux_flexcan_classic_set_timing(const struct device *dev,
					   const struct can_timing *timing)
{
	struct mcux_flexcan_classic_data *data = dev->data;

	return mcux_flexcan_common_set_timing(&data->timing, timing, data->started);
}

static int mcux_flexcan_classic_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES;

	return 0;
}

static int mcux_flexcan_classic_start(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	flexcan_timing_config_t timing;
	int err;

	err = mcux_flexcan_common_check_can_start(config, data->started);

	if (err != 0) {
		return err;
	}

	/* Delay this until start since setting the timing automatically exits freeze mode */
	mcux_flexcan_common_extract_timing_from_can_timing(&timing, &data->timing);
	FLEXCAN_SetTimingConfig(config->base, &timing);

	data->started = true;

	return 0;
}

static int mcux_flexcan_classic_stop(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	can_tx_callback_t function;
	void *arg;
	int alloc;
	int err;

	if (!data->started) {
		return -EALREADY;
	}

	data->started = false;

	/* Abort any pending TX frames before entering freeze mode */
	for (alloc = 0; alloc < MCUX_FLEXCAN_MAX_TX; alloc++) {
		function = data->tx_cbs[alloc].function;
		arg = data->tx_cbs[alloc].arg;

		if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
			FLEXCAN_TransferAbortSend(config->base, &data->handle,
						ALLOC_IDX_TO_TXMB_IDX(alloc));

			function(dev, -ENETDOWN, arg);
			k_sem_give(&data->tx_allocs_sem);
		}
	}

	FLEXCAN_EnterFreezeMode(config->base);

	if (config->phy != NULL) {
		err = can_transceiver_disable(config->phy);
		if (err != 0) {
			LOG_ERR("failed to disable CAN transceiver (err %d)", err);
			return err;
		}
	}

	return 0;
}

static int mcux_flexcan_classic_set_mode(const struct device *dev, can_mode_t mode)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;

	return mcux_flexcan_common_set_can_mode(config, mode, data->started, false);
}

static int mcux_flexcan_classic_get_state(const struct device *dev, enum can_state *state,
					  struct can_bus_err_cnt *err_cnt)
{
	const struct mcux_flexcan_generic_config *config = dev->config;

	struct mcux_flexcan_classic_data *data = dev->data;

	mcux_flexcan_common_get_state(config, data->started, state, err_cnt);
	return 0;
}

static int mcux_flexcan_classic_send(const struct device *dev, const struct can_frame *frame,
				     k_timeout_t timeout, can_tx_callback_t callback,
				     void *user_data)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	flexcan_mb_transfer_t xfer;
	enum can_state state;
	status_t status;
	int alloc;

	__ASSERT_NO_MSG(callback != NULL);

	status = mcux_flexcan_common_verify_can_frame_flags(frame->dlc, frame->flags, frame->id,
							    false);

	if (status != 0) {
		return status;
	}

	if (!data->started) {
		return -ENETDOWN;
	}

	(void)mcux_flexcan_classic_get_state(dev, &state, NULL);
	if (state == CAN_STATE_BUS_OFF) {
		LOG_DBG("Transmit failed, bus-off");
		return -ENETUNREACH;
	}

	if (k_sem_take(&data->tx_allocs_sem, timeout) != 0) {
		return -EAGAIN;
	}

	for (alloc = 0; alloc < MCUX_FLEXCAN_MAX_TX; alloc++) {
		if (!atomic_test_and_set_bit(data->tx_allocs, alloc)) {
			break;
		}
	}

	mcux_flexcan_from_can_frame(frame, &data->tx_cbs[alloc].frame);
	data->tx_cbs[alloc].function = callback;
	data->tx_cbs[alloc].arg = user_data;
	xfer.frame = &data->tx_cbs[alloc].frame;
	xfer.mbIdx = ALLOC_IDX_TO_TXMB_IDX(alloc);
	FLEXCAN_SetTxMbConfig(config->base, xfer.mbIdx, true);
	k_mutex_lock(&data->tx_mutex, K_FOREVER);
	config->irq_disable_func();
	status = FLEXCAN_TransferSendNonBlocking(config->base, &data->handle, &xfer);

	config->irq_enable_func();
	k_mutex_unlock(&data->tx_mutex);
	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

static int mcux_flexcan_classic_add_rx_filter(const struct device *dev,
				      can_rx_callback_t callback,
				      void *user_data,
				      const struct can_filter *filter)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	flexcan_mb_transfer_t xfer;
	status_t status;
	uint32_t mask;
	int alloc = -ENOSPC;
	int i;

	__ASSERT_NO_MSG(callback);

	if (mcux_flexcan_common_verify_frame_filter_flags(false, filter->flags) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->rx_mutex, K_FOREVER);

	/* Find and allocate RX message buffer */
	for (i = RX_START_IDX; i < MCUX_FLEXCAN_MAX_RX; i++) {
		if (!atomic_test_and_set_bit(data->rx_allocs, i)) {
			alloc = i;
			break;
		}
	}

	if (alloc == -ENOSPC) {
		return alloc;
	}

	mcux_flexcan_common_can_filter_to_mbconfig(filter, &data->rx_cbs[alloc].mb_config, &mask);

	data->rx_cbs[alloc].arg = user_data;
	data->rx_cbs[alloc].function = callback;

	FLEXCAN_EnterFreezeMode(config->base);
	config->base->RXIMR[ALLOC_IDX_TO_RXMB_IDX(alloc)] = mask;
	if (data->started) {
		FLEXCAN_ExitFreezeMode(config->base);
	}
	FLEXCAN_SetRxMbConfig(config->base, ALLOC_IDX_TO_RXMB_IDX(alloc),
				&data->rx_cbs[alloc].mb_config, true);
	xfer.frame = &data->rx_cbs[alloc].frame;
	xfer.mbIdx = ALLOC_IDX_TO_RXMB_IDX(alloc);
	status = FLEXCAN_TransferReceiveNonBlocking(config->base, &data->handle,
						      &xfer);

	if (status != kStatus_Success) {
		LOG_ERR("Failed to start rx for filter id %d (err = %d)",
			alloc, status);
		alloc = -ENOSPC;
	}

	k_mutex_unlock(&data->rx_mutex);

	return alloc;
}

static void mcux_flexcan_classic_set_state_change_callback(const struct device *dev,
						   can_state_change_callback_t callback,
						   void *user_data)
{
	struct mcux_flexcan_classic_data *data = dev->data;

	data->state_change_cb = callback;
	data->state_change_cb_data = user_data;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static int mcux_flexcan_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	enum can_state state;
	uint64_t start_time;
	int ret = 0;

	if (!data->started) {
		return -ENETDOWN;
	}

	(void)mcux_flexcan_classic_get_state(dev, &state, NULL);
	if (state != CAN_STATE_BUS_OFF) {
		return 0;
	}

	start_time = k_uptime_ticks();
	config->base->CTRL1 &= ~CAN_CTRL1_BOFFREC_MASK;

	if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		(void)mcux_flexcan_classic_get_state(dev, &state, NULL);

		while (state == CAN_STATE_BUS_OFF) {
			if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
			    k_uptime_ticks() - start_time >= timeout.ticks) {
				ret = -EAGAIN;
			}

			(void)mcux_flexcan_classic_get_state(dev, &state, NULL);
		}
	}

	config->base->CTRL1 |= CAN_CTRL1_BOFFREC_MASK;

	return ret;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

static void mcux_flexcan_classic_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;

	if (filter_id >= MCUX_FLEXCAN_MAX_RX) {
		LOG_ERR("Detach: Filter id >= MAX_RX (%d >= %d)", filter_id,
			MCUX_FLEXCAN_MAX_RX);
		return;
	}

	k_mutex_lock(&data->rx_mutex, K_FOREVER);

	if (atomic_test_and_clear_bit(data->rx_allocs, filter_id)) {
		FLEXCAN_TransferAbortReceive(config->base, &data->handle,
					     ALLOC_IDX_TO_RXMB_IDX(filter_id));
		FLEXCAN_SetRxMbConfig(config->base,
				      ALLOC_IDX_TO_RXMB_IDX(filter_id), NULL,
				      false);
		data->rx_cbs[filter_id].function = NULL;
		data->rx_cbs[filter_id].arg = NULL;
	} else {
		LOG_WRN("Filter ID %d already detached", filter_id);
	}

	k_mutex_unlock(&data->rx_mutex);
}

static inline void mcux_flexcan_classic_transfer_error_status(const struct device *dev,
						      uint64_t error)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	const can_state_change_callback_t cb = data->state_change_cb;
	void *cb_data = data->state_change_cb_data;
	can_tx_callback_t function;
	void *arg;
	int alloc;
	enum can_state state;
	struct can_bus_err_cnt err_cnt;

	increment_error_counters(dev, error);

	(void)mcux_flexcan_classic_get_state(dev, &state, &err_cnt);
	if (data->state != state) {
		data->state = state;

		if (cb != NULL) {
			cb(dev, state, err_cnt, cb_data);
		}
	}

	if (state == CAN_STATE_BUS_OFF) {
		/* Abort any pending TX frames in case of bus-off */
		for (alloc = 0; alloc < MCUX_FLEXCAN_MAX_TX; alloc++) {
			/* Copy callback function and argument before clearing bit */
			function = data->tx_cbs[alloc].function;
			arg = data->tx_cbs[alloc].arg;

			if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
				FLEXCAN_TransferAbortSend(config->base, &data->handle,
							  ALLOC_IDX_TO_TXMB_IDX(alloc));

				function(dev, -ENETUNREACH, arg);
				k_sem_give(&data->tx_allocs_sem);
			}
		}
	}
}

static inline void mcux_flexcan_classic_transfer_tx_idle(const struct device *dev,
						 uint32_t mb)
{
	struct mcux_flexcan_classic_data *data = dev->data;
	can_tx_callback_t function;
	void *arg;
	int alloc;

	alloc = TX_MBIDX_TO_ALLOC_IDX(mb);

	/* Copy callback function and argument before clearing bit */
	function = data->tx_cbs[alloc].function;
	arg = data->tx_cbs[alloc].arg;

	if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
		function(dev, 0, arg);
		k_sem_give(&data->tx_allocs_sem);
	}
}

static inline void mcux_flexcan_classic_transfer_rx_idle(const struct device *dev,
						 uint32_t mb)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	can_rx_callback_t function;
	flexcan_mb_transfer_t xfer;
	struct can_frame frame;
	status_t status;
	void *arg;
	int alloc;

	alloc = RX_MBIDX_TO_ALLOC_IDX(mb);
	function = data->rx_cbs[alloc].function;
	arg = data->rx_cbs[alloc].arg;

	if (atomic_test_bit(data->rx_allocs, alloc)) {
		mcux_flexcan_to_can_frame(&data->rx_cbs[alloc].frame, &frame);
		function(dev, &frame, arg);

		/* Setup RX message buffer to receive next message */
		xfer.frame = &data->rx_cbs[alloc].frame;
		xfer.mbIdx = mb;
		status = FLEXCAN_TransferReceiveNonBlocking(config->base,
							    &data->handle,
							    &xfer);
		if (status != kStatus_Success) {
			LOG_ERR("Failed to restart rx for filter id %d "
				"(err = %d)", alloc, status);
		}
	}
}

static FLEXCAN_CALLBACK(mcux_flexcan_transfer_callback)
{
	struct mcux_flexcan_classic_data *data = (struct mcux_flexcan_classic_data *)userData;
	const struct mcux_flexcan_generic_config *config = data->dev->config;
	/*
	 * The result field can either be a MB index (which is limited to 32 bit
	 * value) or a status flags value, which is 32 bit on some platforms but
	 * 64 on others. To decouple the remaining functions from this, the
	 * result field is always promoted to uint64_t.
	 */
	uint32_t mb = (uint32_t)result;
	uint64_t status_flags = result;

	ARG_UNUSED(base);

	switch (status) {
	case kStatus_FLEXCAN_UnHandled:
		/* Not all fault confinement state changes are handled by the HAL */
		__fallthrough;
	case kStatus_FLEXCAN_ErrorStatus:
		mcux_flexcan_classic_transfer_error_status(data->dev, status_flags);
		break;
	case kStatus_FLEXCAN_TxSwitchToRx:
		FLEXCAN_TransferAbortReceive(config->base, &data->handle, mb);
		__fallthrough;
	case kStatus_FLEXCAN_TxIdle:
		mcux_flexcan_classic_transfer_tx_idle(data->dev, mb);
		break;
	case kStatus_FLEXCAN_RxOverflow:
		CAN_STATS_RX_OVERRUN_INC(data->dev);
		__fallthrough;
	case kStatus_Fail:
		/* If reading an RX MB failed mark it as idle to be reprocessed. */
		__fallthrough;
	case kStatus_FLEXCAN_RxIdle:
		mcux_flexcan_classic_transfer_rx_idle(data->dev, mb);
		break;
	default:
		LOG_WRN("Unhandled status 0x%08x (result = 0x%016llx)",
			status, status_flags);
	}
}

static void mcux_flexcan_isr(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;

	FLEXCAN_TransferHandleIRQ(config->base, &data->handle);
}

static int mcux_flexcan_init(const struct device *dev)
{
	const struct mcux_flexcan_generic_config *config = dev->config;
	struct mcux_flexcan_classic_data *data = dev->data;
	flexcan_config_t flexcan_config;
	uint32_t clock_freq;
	int err;

	if (config->phy != NULL) {
		if (!device_is_ready(config->phy)) {
			LOG_ERR("CAN transceiver not ready");
			return -ENODEV;
		}
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->rx_mutex);
	k_mutex_init(&data->tx_mutex);
	k_sem_init(&data->tx_allocs_sem, MCUX_FLEXCAN_MAX_TX,
		   MCUX_FLEXCAN_MAX_TX);

	data->timing.sjw = config->sjw;
	if (config->sample_point && USE_SP_ALGO) {
		err = mcux_flexcan_common_calc_timing(dev, &data->timing,
						      config->bitrate, config->sample_point);
		if (err != 0) {
			return -EIO;
		}
	} else {
		mcux_flexcan_common_config_calc_bitrate(dev, config, &data->timing);
	}

#ifdef CONFIG_PINCTRL
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
#endif

	err = mcux_flexcan_common_get_core_clock(dev, &clock_freq);
	if (err != 0) {
		return -EIO;
	}

	data->dev = dev;

	mcux_flexcan_common_init_config(&flexcan_config, &data->timing, clock_freq,
					config->clk_source,
					FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(0));

	/* Initialize in listen-only mode since FLEXCAN_Init() exits freeze mode */
	FLEXCAN_Init(config->base, &flexcan_config, clock_freq);
	FLEXCAN_TransferCreateHandle(config->base, &data->handle,
				     mcux_flexcan_transfer_callback, data);

	/* Manually enter freeze mode, set normal mode, and clear error counters */
	FLEXCAN_EnterFreezeMode(config->base);
	(void)mcux_flexcan_classic_set_mode(dev, CAN_MODE_NORMAL);
	config->base->ECR &= ~(CAN_ECR_TXERRCNT_MASK | CAN_ECR_RXERRCNT_MASK);

	config->irq_config_func(dev);

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	config->base->CTRL1 |= CAN_CTRL1_BOFFREC_MASK;
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

	(void)mcux_flexcan_classic_get_state(dev, &data->state, NULL);

	return 0;
}

static const struct can_driver_api mcux_flexcan_driver_api = {
	.get_capabilities = mcux_flexcan_classic_get_capabilities,
	.start = mcux_flexcan_classic_start,
	.stop = mcux_flexcan_classic_stop,
	.set_mode = mcux_flexcan_classic_set_mode,
	.set_timing = mcux_flexcan_classic_set_timing,
	.send = mcux_flexcan_classic_send,
	.add_rx_filter = mcux_flexcan_classic_add_rx_filter,
	.remove_rx_filter = mcux_flexcan_classic_remove_rx_filter,
	.get_state = mcux_flexcan_classic_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = mcux_flexcan_recover,
#endif
	.set_state_change_callback = mcux_flexcan_classic_set_state_change_callback,
	.get_core_clock = mcux_flexcan_common_get_core_clock,
	.get_max_filters = mcux_flexcan_common_get_max_filters,
	.get_max_bitrate = mcux_flexcan_common_get_max_bitrate,
	/*
	 * FlexCAN timing limits are specified in the "FLEXCANx_CTRL1 field
	 * descriptions" table in the SoC reference manual.
	 *
	 * Note that the values here are the "physical" timing limits, whereas
	 * the register field limits are physical values minus 1 (which is
	 * handled by the flexcan_config_t field assignments elsewhere in this
	 * driver).
	 */
	.timing_min = {
		.sjw = 0x01,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x02,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x04,
		.prop_seg = 0x08,
		.phase_seg1 = 0x08,
		.phase_seg2 = 0x08,
		.prescaler = 0x100
	}
};

#define FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, cell) \
	DT_IRQ_BY_NAME(node_id, \
		DT_STRING_TOKEN_BY_IDX(node_id, prop, idx), cell)

#define FLEXCAN_IRQ_ENABLE_CODE(node_id, prop, idx) \
	irq_enable(FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, irq));

#define FLEXCAN_IRQ_DISABLE_CODE(node_id, prop, idx) \
	irq_disable(FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, irq));

#define FLEXCAN_IRQ_CONFIG_CODE(node_id, prop, idx) \
	do {								\
		IRQ_CONNECT(FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, irq), \
		FLEXCAN_IRQ_BY_IDX(node_id, prop, idx, priority), \
		mcux_flexcan_isr, \
		DEVICE_DT_GET(node_id), 0); \
		FLEXCAN_IRQ_ENABLE_CODE(node_id, prop, idx); \
	} while (false);

#ifdef CONFIG_PINCTRL
#define FLEXCAN_PINCTRL_DEFINE(id) PINCTRL_DT_INST_DEFINE(id);
#define FLEXCAN_PINCTRL_INIT(id) .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),
#else
#define FLEXCAN_PINCTRL_DEFINE(id)
#define FLEXCAN_PINCTRL_INIT(id)
#endif /* CONFIG_PINCTRL */


#define FLEXCAN_DEVICE_INIT_MCUX(id)					\
	FLEXCAN_PINCTRL_DEFINE(id)					\
									\
	static void mcux_flexcan_irq_config_##id(const struct device *dev); \
	static void mcux_flexcan_irq_enable_##id(void); \
	static void mcux_flexcan_irq_disable_##id(void); \
									\
	static const struct mcux_flexcan_generic_config mcux_flexcan_generic_config_##id = { \
		.base = (CAN_Type *)DT_INST_REG_ADDR(id),		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(id)),	\
		.clock_subsys = (clock_control_subsys_t)		\
			DT_INST_CLOCKS_CELL(id, name),			\
		.clk_source = DT_INST_PROP(id, clk_source),		\
		.bitrate = DT_INST_PROP(id, bus_speed),			\
		.sjw = DT_INST_PROP(id, sjw),				\
		.prop_seg = DT_INST_PROP_OR(id, prop_seg, 0),		\
		.phase_seg1 = DT_INST_PROP_OR(id, phase_seg1, 0),	\
		.phase_seg2 = DT_INST_PROP_OR(id, phase_seg2, 0),	\
		.sample_point = DT_INST_PROP_OR(id, sample_point, 0),	\
		.irq_config_func = mcux_flexcan_irq_config_##id,	\
		.irq_enable_func = mcux_flexcan_irq_enable_##id,	\
		.irq_disable_func = mcux_flexcan_irq_disable_##id,	\
		.phy = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(id, phys)), \
		.max_bitrate = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(id, 1000000), \
		FLEXCAN_PINCTRL_INIT(id)				\
	};                                                         \
                                                                                                   \
	static struct mcux_flexcan_classic_data mcux_flexcan_data_##id;                            \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(id, mcux_flexcan_init,		\
				  NULL, &mcux_flexcan_data_##id,	\
				  &mcux_flexcan_generic_config_##id,		\
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
				  &mcux_flexcan_driver_api);		\
									\
	static void mcux_flexcan_irq_config_##id(const struct device *dev) \
	{								\
		DT_INST_FOREACH_PROP_ELEM(id, interrupt_names, FLEXCAN_IRQ_CONFIG_CODE); \
	} \
									\
	static void mcux_flexcan_irq_enable_##id(void) \
	{								\
		DT_INST_FOREACH_PROP_ELEM(id, interrupt_names, FLEXCAN_IRQ_ENABLE_CODE); \
	} \
									\
	static void mcux_flexcan_irq_disable_##id(void) \
	{								\
		DT_INST_FOREACH_PROP_ELEM(id, interrupt_names, FLEXCAN_IRQ_DISABLE_CODE); \
	}

DT_INST_FOREACH_STATUS_OKAY(FLEXCAN_DEVICE_INIT_MCUX)
