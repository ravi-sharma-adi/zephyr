/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <sys/byteorder.h>

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "ticker/ticker.h"

#include "pdu.h"

#include "lll.h"
#include "lll/lll_vendor.h"
#include "lll_clock.h"
#include "lll_scan.h"
#include "lll_sync.h"
#include "lll_sync_iso.h"

#include "ull_scan_types.h"
#include "ull_sync_types.h"

#include "ull_internal.h"
#include "ull_scan_internal.h"
#include "ull_sync_internal.h"
#include "ull_sync_iso_internal.h"

#include "ll.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME bt_ctlr_ull_sync_iso
#include "common/log.h"
#include "hal/debug.h"

static int init_reset(void);
static inline struct ll_sync_iso_set *sync_iso_acquire(void);
static void timeout_cleanup(struct ll_sync_iso_set *sync_iso);
static void ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
		      uint32_t remainder, uint16_t lazy, uint8_t force,
		      void *param);
static void ticker_start_op_cb(uint32_t status, void *param);
static void ticker_update_op_cb(uint32_t status, void *param);
static void ticker_stop_op_cb(uint32_t status, void *param);
static void sync_lost(void *param);

static memq_link_t link_lll_prepare;
static struct mayfly mfy_lll_prepare = {0, 0, &link_lll_prepare, NULL, NULL};

static struct ll_sync_iso_set ll_sync_iso[CONFIG_BT_CTLR_SCAN_SYNC_ISO_SET];
static void *sync_iso_free;

uint8_t ll_big_sync_create(uint8_t big_handle, uint16_t sync_handle,
			   uint8_t encryption, uint8_t *bcode, uint8_t mse,
			   uint16_t sync_timeout, uint8_t num_bis,
			   uint8_t *bis)
{
	struct ll_sync_iso_set *sync_iso;
	memq_link_t *link_sync_estab;
	memq_link_t *link_sync_lost;
	struct node_rx_hdr *node_rx;
	struct ll_sync_set *sync;
	struct lll_sync_iso *lll;

	sync = ull_sync_is_enabled_get(sync_handle);
	if (!sync || sync->iso.sync_iso) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	link_sync_estab = ll_rx_link_alloc();
	if (!link_sync_estab) {
		return BT_HCI_ERR_MEM_CAPACITY_EXCEEDED;
	}

	link_sync_lost = ll_rx_link_alloc();
	if (!link_sync_lost) {
		ll_rx_link_release(link_sync_estab);

		return BT_HCI_ERR_MEM_CAPACITY_EXCEEDED;
	}

	node_rx = ll_rx_alloc();
	if (!node_rx) {
		ll_rx_link_release(link_sync_lost);
		ll_rx_link_release(link_sync_estab);

		return BT_HCI_ERR_MEM_CAPACITY_EXCEEDED;
	}

	sync_iso = sync_iso_acquire();
	if (!sync_iso) {
		ll_rx_release(node_rx);
		ll_rx_link_release(link_sync_lost);
		ll_rx_link_release(link_sync_estab);

		return BT_HCI_ERR_MEM_CAPACITY_EXCEEDED;
	}

	/* Initialize the ISO sync ULL context */
	sync_iso->sync = sync;
	sync_iso->timeout = sync_timeout;
	sync_iso->timeout_reload = 0U;
	sync_iso->timeout_expire = 0U;

	/* Setup the periodic sync to establish ISO sync */
	node_rx->link = link_sync_estab;
	sync->iso.node_rx_estab = node_rx;
	sync_iso->node_rx_lost.hdr.link = link_sync_lost;

	/* Initialize sync LLL context */
	lll = &sync_iso->lll;
	lll->latency_prepare = 0U;
	lll->latency_event = 0U;
	lll->data_chan_id = 0U;
	lll->window_widening_prepare_us = 0U;
	lll->window_widening_event_us = 0U;

	/* Initialize ULL and LLL headers */
	ull_hdr_init(&sync_iso->ull);
	lll_hdr_init(lll, sync_iso);

	/* Enable periodic advertising to establish ISO sync */
	sync->iso.sync_iso = sync_iso;

	return BT_HCI_ERR_SUCCESS;
}

uint8_t ll_big_sync_terminate(uint8_t big_handle, void **rx)
{
	struct ll_sync_iso_set *sync_iso;
	memq_link_t *link_sync_estab;
	struct node_rx_pdu *node_rx;
	memq_link_t *link_sync_lost;
	struct ll_sync_set *sync;
	int err;

	sync_iso = ull_sync_iso_get(big_handle);
	if (!sync_iso) {
		return BT_HCI_ERR_UNKNOWN_ADV_IDENTIFIER;
	}

	sync = sync_iso->sync;
	if (sync && sync->iso.sync_iso) {
		struct node_rx_sync_iso *se;

		if (sync->iso.sync_iso != sync_iso) {
			return BT_HCI_ERR_CMD_DISALLOWED;
		}
		sync->iso.sync_iso = NULL;
		sync_iso->sync = NULL;

		node_rx = (void *)sync->iso.node_rx_estab;
		link_sync_estab = node_rx->hdr.link;
		link_sync_lost = sync_iso->node_rx_lost.hdr.link;

		ll_rx_link_release(link_sync_lost);
		ll_rx_link_release(link_sync_estab);
		ll_rx_release(node_rx);

		node_rx = (void *)&sync_iso->node_rx_lost;
		node_rx->hdr.type = NODE_RX_TYPE_SYNC_ISO;
		node_rx->hdr.handle = 0xffff;

		/* NOTE: Since NODE_RX_TYPE_SYNC_ISO is only generated from ULL
		 *       context, pass ULL context as parameter.
		 */
		node_rx->hdr.rx_ftr.param = sync_iso;

		/* NOTE: struct node_rx_lost has uint8_t member following the
		 *       struct node_rx_hdr to store the reason.
		 */
		se = (void *)node_rx->pdu;
		se->status = BT_HCI_ERR_OP_CANCELLED_BY_HOST;

		*rx = node_rx;

		return BT_HCI_ERR_SUCCESS;
	}

	err = ull_ticker_stop_with_mark((TICKER_ID_SCAN_SYNC_ISO_BASE +
					 big_handle), sync_iso, &sync_iso->lll);
	LL_ASSERT(err == 0 || err == -EALREADY);
	if (err) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	link_sync_lost = sync_iso->node_rx_lost.hdr.link;
	ll_rx_link_release(link_sync_lost);

	ull_sync_iso_release(sync_iso);

	return BT_HCI_ERR_SUCCESS;
}

int ull_sync_iso_init(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int ull_sync_iso_reset(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

struct ll_sync_iso_set *ull_sync_iso_get(uint8_t handle)
{
	if (handle >= CONFIG_BT_CTLR_SCAN_SYNC_ISO_SET) {
		return NULL;
	}

	return &ll_sync_iso[handle];
}

uint8_t ull_sync_iso_handle_get(struct ll_sync_iso_set *sync)
{
	return mem_index_get(sync, ll_sync_iso,
			     sizeof(struct ll_sync_iso_set));
}

uint8_t ull_sync_iso_lll_handle_get(struct lll_sync_iso *lll)
{
	return ull_sync_handle_get(HDR_LLL2ULL(lll));
}

void ull_sync_iso_release(struct ll_sync_iso_set *sync_iso)
{
	mem_release(sync_iso, &sync_iso_free);
}

void ull_sync_iso_setup(struct ll_sync_iso_set *sync_iso,
			struct node_rx_hdr *node_rx,
			uint8_t *acad, uint8_t acad_len)
{
	uint32_t ticks_slot_overhead;
	uint32_t sync_iso_offset_us;
	uint32_t ticks_slot_offset;
	struct lll_sync_iso *lll;
	struct node_rx_ftr *ftr;
	struct pdu_big_info *bi;
	uint32_t ready_delay_us;
	uint32_t interval_us;
	struct pdu_adv *pdu;
	uint16_t interval;
	uint8_t bi_size;
	uint8_t handle;
	uint32_t ret;
	uint8_t sca;

	if (acad_len < (PDU_BIG_INFO_CLEARTEXT_SIZE + 2)) {
		return;
	}

	/* TODO: Parse and find the BIGInfo */
	bi_size = acad[0];
	bi = (void *)&acad[2];

	lll = &sync_iso->lll;
	memcpy(lll->seed_access_addr, &bi->seed_access_addr,
	       sizeof(lll->seed_access_addr));
	memcpy(lll->base_crc_init, &bi->base_crc_init,
	       sizeof(lll->base_crc_init));

	memcpy(lll->data_chan_map, bi->chm_phy, sizeof(lll->data_chan_map));
	lll->data_chan_map[4] &= ~0xE0;
	lll->data_chan_count = util_ones_count_get(&lll->data_chan_map[0],
						   sizeof(lll->data_chan_map));
	if (lll->data_chan_count < 2) {
		return;
	}

	/* Reset ISO create BIG flag in the periodic advertising context */
	sync_iso->sync->iso.sync_iso = NULL;

	lll->phy = BIT(bi->chm_phy[4] >> 5);

	lll->num_bis = bi->num_bis;
	lll->bn = bi->bn;
	lll->nse = bi->nse;
	lll->sub_interval = bi->sub_interval;
	lll->max_pdu = bi->max_pdu;
	lll->pto = bi->pto;
	lll->bis_spacing = bi->spacing;
	lll->irc = bi->irc;
	lll->sdu_interval = bi->sdu_interval;

	lll->payload_count = (uint64_t)bi->payload_count_framing[0];
	lll->payload_count |= (uint64_t)bi->payload_count_framing[1] << 8;
	lll->payload_count |= (uint64_t)bi->payload_count_framing[2] << 16;
	lll->payload_count |= (uint64_t)bi->payload_count_framing[3] << 24;
	lll->payload_count |= (uint64_t)bi->payload_count_framing[4] << 32;

	interval = sys_le16_to_cpu(bi->iso_interval);
	interval_us = interval * CONN_INT_UNIT_US;

	sync_iso->timeout_reload =
		RADIO_SYNC_EVENTS((sync_iso->timeout * 10U * 1000U),
				  interval_us);

	sca = sync_iso->sync->lll.sca;
	lll->window_widening_periodic_us =
		(((lll_clock_ppm_local_get() + lll_clock_ppm_get(sca)) *
		  interval_us) + (1000000 - 1)) / 1000000U;
	lll->window_widening_max_us = (interval_us >> 1) - EVENT_IFS_US;
	if (bi->offs_units) {
		lll->window_size_event_us = 300U;
	} else {
		lll->window_size_event_us = 30U;
	}

	ftr = &node_rx->rx_ftr;
	pdu = (void *)((struct node_rx_pdu *)node_rx)->pdu;

	ready_delay_us = lll_radio_rx_ready_delay_get(lll->phy, 1);

	sync_iso_offset_us = ftr->radio_end_us;
	sync_iso_offset_us += (uint32_t)bi->offs * lll->window_size_event_us;
	sync_iso_offset_us -= PKT_AC_US(pdu->len, lll->phy);
	sync_iso_offset_us -= EVENT_OVERHEAD_START_US;
	sync_iso_offset_us -= EVENT_JITTER_US;
	sync_iso_offset_us -= ready_delay_us;

	interval_us -= lll->window_widening_periodic_us;

	/* TODO: active_to_start feature port */
	sync_iso->ull.ticks_active_to_start = 0U;
	sync_iso->ull.ticks_prepare_to_start =
		HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_XTAL_US);
	sync_iso->ull.ticks_preempt_to_start =
		HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_PREEMPT_MIN_US);
	sync_iso->ull.ticks_slot =
		HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US +
				       ready_delay_us +
				       PKT_AC_US(PDU_AC_EXT_PAYLOAD_SIZE_MAX,
						 lll->phy) +
				       EVENT_OVERHEAD_END_US);

	ticks_slot_offset = MAX(sync_iso->ull.ticks_active_to_start,
				sync_iso->ull.ticks_prepare_to_start);

	if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT)) {
		ticks_slot_overhead = ticks_slot_offset;
	} else {
		ticks_slot_overhead = 0U;
	}

	/* setup to use ISO create prepare function until sync established */
	mfy_lll_prepare.fp = lll_sync_iso_create_prepare;

	handle = ull_sync_iso_handle_get(sync_iso);
	ret = ticker_start(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_ULL_HIGH,
			   (TICKER_ID_SCAN_SYNC_ISO_BASE + handle),
			   ftr->ticks_anchor - ticks_slot_offset,
			   HAL_TICKER_US_TO_TICKS(sync_iso_offset_us),
			   HAL_TICKER_US_TO_TICKS(interval_us),
			   HAL_TICKER_REMAINDER(interval_us),
			   TICKER_NULL_LAZY,
			   (sync_iso->ull.ticks_slot + ticks_slot_overhead),
			   ticker_cb, sync_iso,
			   ticker_start_op_cb, (void *)__LINE__);
	LL_ASSERT((ret == TICKER_STATUS_SUCCESS) ||
		  (ret == TICKER_STATUS_BUSY));
}

void ull_sync_iso_estab_done(struct node_rx_event_done *done)
{
	struct ll_sync_iso_set *sync_iso;
	struct lll_sync_iso *lll;
	struct node_rx_hdr *rx;

	/* switch to normal prepare */
	mfy_lll_prepare.fp = lll_sync_iso_prepare;

	/* Get reference to ULL context */
	sync_iso = CONTAINER_OF(done->param, struct ll_sync_iso_set, ull);
	lll = &sync_iso->lll;

	/* Prepare BIG Sync Established */
	rx = sync_iso->sync->iso.node_rx_estab;
	rx->type = NODE_RX_TYPE_SYNC_ISO;
	rx->handle = ull_sync_iso_handle_get(sync_iso);
	rx->rx_ftr.param = sync_iso;

	ll_rx_put(rx->link, rx);
	ll_rx_sched();

	ull_sync_iso_done(done);
}

void ull_sync_iso_done(struct node_rx_event_done *done)
{
	struct ll_sync_iso_set *sync_iso;
	uint32_t ticks_drift_minus;
	uint32_t ticks_drift_plus;
	struct lll_sync_iso *lll;
	uint16_t elapsed_event;
	uint16_t latency_event;
	uint16_t lazy;
	uint8_t force;

	/* Get reference to ULL context */
	sync_iso = CONTAINER_OF(done->param, struct ll_sync_iso_set, ull);
	lll = &sync_iso->lll;

	/* Events elapsed used in timeout checks below */
	latency_event = lll->latency_event;
	elapsed_event = latency_event + 1;

	/* Sync drift compensation and new skip calculation
	 */
	ticks_drift_plus = 0U;
	ticks_drift_minus = 0U;
	if (done->extra.trx_cnt) {
		/* Calculate drift in ticks unit */
		ull_drift_ticks_get(done, &ticks_drift_plus,
				    &ticks_drift_minus);

		/* Reset latency */
		lll->latency_event = 0U;
	}

	/* Reset supervision countdown */
	if (done->extra.crc_valid) {
		sync_iso->timeout_expire = 0U;
	}

	/* if anchor point not sync-ed, start timeout countdown, and break
	 * latency if any.
	 */
	else {
		if (!sync_iso->timeout_expire) {
			sync_iso->timeout_expire = sync_iso->timeout_reload;
		}
	}

	/* check timeout */
	force = 0U;
	if (sync_iso->timeout_expire) {
		if (sync_iso->timeout_expire > elapsed_event) {
			sync_iso->timeout_expire -= elapsed_event;

			/* break skip */
			lll->latency_event = 0U;

			if (latency_event) {
				force = 1U;
			}
		} else {
			timeout_cleanup(sync_iso);

			return;
		}
	}

	/* check if skip needs update */
	lazy = 0U;
	if ((force) || (latency_event != lll->latency_event)) {
		lazy = lll->latency_event + 1U;
	}

	/* Update Sync ticker instance */
	if (ticks_drift_plus || ticks_drift_minus || lazy || force) {
		uint8_t handle = ull_sync_iso_handle_get(sync_iso);
		uint32_t ticker_status;

		/* Call to ticker_update can fail under the race
		 * condition where in the periodic sync role is being stopped
		 * but at the same time it is preempted by periodic sync event
		 * that gets into close state. Accept failure when periodic sync
		 * role is being stopped.
		 */
		ticker_status = ticker_update(TICKER_INSTANCE_ID_CTLR,
					      TICKER_USER_ID_ULL_HIGH,
					      (TICKER_ID_SCAN_SYNC_ISO_BASE +
					       handle),
					      ticks_drift_plus,
					      ticks_drift_minus, 0, 0,
					      lazy, force,
					      ticker_update_op_cb,
					      sync_iso);
		LL_ASSERT((ticker_status == TICKER_STATUS_SUCCESS) ||
			  (ticker_status == TICKER_STATUS_BUSY) ||
			  ((void *)sync_iso == ull_disable_mark_get()));
	}
}

static int init_reset(void)
{
	/* Initialize sync pool. */
	mem_init(ll_sync_iso, sizeof(struct ll_sync_iso_set),
		 sizeof(ll_sync_iso) / sizeof(struct ll_sync_iso_set),
		 &sync_iso_free);

	return 0;
}

static inline struct ll_sync_iso_set *sync_iso_acquire(void)
{
	return mem_acquire(&sync_iso_free);
}

static void timeout_cleanup(struct ll_sync_iso_set *sync_iso)
{
	uint16_t handle = ull_sync_iso_handle_get(sync_iso);
	uint32_t ret;

	/* Stop Periodic Sync Ticker */
	ret = ticker_stop(TICKER_INSTANCE_ID_CTLR, TICKER_USER_ID_ULL_HIGH,
			  (TICKER_ID_SCAN_SYNC_ISO_BASE + handle),
			  ticker_stop_op_cb, (void *)sync_iso);
	LL_ASSERT((ret == TICKER_STATUS_SUCCESS) ||
		  (ret == TICKER_STATUS_BUSY));
}

static void ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
		      uint32_t remainder, uint16_t lazy, uint8_t force,
		      void *param)
{
	static struct lll_prepare_param p;
	struct ll_sync_iso_set *sync_iso = param;
	struct lll_sync_iso *lll;
	uint32_t ret;
	uint8_t ref;

	DEBUG_RADIO_PREPARE_O(1);

	lll = &sync_iso->lll;

	/* Increment prepare reference count */
	ref = ull_ref_inc(&sync_iso->ull);
	LL_ASSERT(ref);

	/* Append timing parameters */
	p.ticks_at_expire = ticks_at_expire;
	p.remainder = remainder;
	p.lazy = lazy;
	p.force = force;
	p.param = lll;
	mfy_lll_prepare.param = &p;

	/* Kick LLL prepare */
	ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL, 0,
			     &mfy_lll_prepare);
	LL_ASSERT(!ret);

	DEBUG_RADIO_PREPARE_O(1);
}

static void ticker_start_op_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	LL_ASSERT(status == TICKER_STATUS_SUCCESS);
}

static void ticker_update_op_cb(uint32_t status, void *param)
{
	LL_ASSERT(status == TICKER_STATUS_SUCCESS ||
		  param == ull_disable_mark_get());
}

static void ticker_stop_op_cb(uint32_t status, void *param)
{
	uint32_t retval;
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, sync_lost};

	LL_ASSERT(status == TICKER_STATUS_SUCCESS);

	mfy.param = param;

	retval = mayfly_enqueue(TICKER_USER_ID_ULL_LOW, TICKER_USER_ID_ULL_HIGH,
				0, &mfy);
	LL_ASSERT(!retval);
}

static void sync_lost(void *param)
{
	struct ll_sync_iso_set *sync_iso = param;
	struct node_rx_pdu *rx;

	/* Generate BIG sync lost */
	rx = (void *)&sync_iso->node_rx_lost;
	rx->hdr.handle = ull_sync_iso_handle_get(sync_iso);
	rx->hdr.type = NODE_RX_TYPE_SYNC_ISO_LOST;
	rx->hdr.rx_ftr.param = sync_iso;

	/* Enqueue the BIG sync lost towards ULL context */
	ll_rx_put(rx->hdr.link, rx);
	ll_rx_sched();
}
