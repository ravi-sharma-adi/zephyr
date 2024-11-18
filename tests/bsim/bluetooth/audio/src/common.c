/*
 * Copyright (c) 2019 Bose Corporation
 * Copyright (c) 2020-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic_types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "bs_cmd_line.h"
#include "bs_dynargs.h"
#include "bs_pc_backchannel.h"
#include "posix_native_task.h"
#include "bs_types.h"
#include "bsim_args_runner.h"
#include "bstests.h"
#include "common.h"

extern enum bst_result_t bst_result;
struct bt_conn *default_conn;
atomic_t flag_connected;
atomic_t flag_disconnected;
atomic_t flag_conn_updated;
volatile bt_security_t security_level;

const struct bt_data ad[AD_SIZE] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))
};

static void device_found(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad_buf)
{
	/* We use BT_BAP_COEX_INT_MS_7_5_10_FAST to best support peripherals
	 * that support both 7.5 and 10ms SDU interval
	 */
	const struct bt_le_conn_param *conn_param =
		BT_LE_CONN_PARAM(BT_GAP_MS_TO_CONN_INTERVAL(BT_BAP_COEX_INT_MS_7_5_10_FAST),
				 BT_GAP_MS_TO_CONN_INTERVAL(BT_BAP_COEX_INT_MS_7_5_10_FAST), 0,
				 BT_GAP_MS_TO_CONN_TIMEOUT(4000));
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if ((info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) == 0) {
		return;
	}

	bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, info->rssi);

	/* connect only to devices in close proximity */
	if (info->rssi < -70) {
		FAIL("RSSI too low");
		return;
	}

	printk("Stopping scan\n");
	if (bt_le_scan_stop()) {
		FAIL("Could not stop scan");
		return;
	}

	err = bt_conn_le_create(info->addr, BT_CONN_LE_CREATE_CONN, conn_param, &default_conn);
	if (err) {
		FAIL("Could not connect to peer: %d", err);
	}
}

struct bt_le_scan_cb common_scan_cb = {
	.recv = device_found,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (default_conn == NULL) {
		default_conn = bt_conn_ref(conn);
	}

	if (err != 0) {
		bt_conn_unref(default_conn);
		default_conn = NULL;

		FAIL("Failed to connect to %s (0x%02x)\n", addr, err);
		return;
	}

	printk("Connected to %s (%p)\n", addr, conn);
	SET_FLAG(flag_connected);
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	UNSET_FLAG(flag_connected);
	UNSET_FLAG(flag_conn_updated);
	SET_FLAG(flag_disconnected);

	security_level = BT_SECURITY_L1;
}

static void conn_param_updated_cb(struct bt_conn *conn, uint16_t interval, uint16_t latency,
				  uint16_t timeout)
{
	printk("Connection parameter updated: %p 0x%04X (%u us), 0x%04X, 0x%04X\n", conn, interval,
	       BT_CONN_INTERVAL_TO_US(interval), latency, timeout);

	SET_FLAG(flag_conn_updated);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	printk("Security changed: %p level %d err %d\n", conn, level, err);

	if (err == BT_SECURITY_ERR_SUCCESS) {
		security_level = level;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = conn_param_updated_cb,
	.security_changed = security_changed_cb,
};

void setup_broadcast_adv(struct bt_le_ext_adv **adv)
{
	/* Zephyr Controller works best while Extended Advertising interval to be a multiple
	 * of the ISO Interval minus 10 ms (max. advertising random delay). This is
	 * required to place the AUX_ADV_IND PDUs in a non-overlapping interval with the
	 * Broadcast ISO radio events.
	 */
	const struct bt_le_adv_param *ext_adv_param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_EXT_ADV, BT_GAP_MS_TO_ADV_INTERVAL(BT_BAP_COEX_INT_MS_10_FAST_2 - 10),
		BT_GAP_MS_TO_ADV_INTERVAL(BT_BAP_COEX_INT_MS_10_FAST_2 - 10), NULL);

	/* We use BT_BAP_COEX_INT_MS_10_FAST_2 as this sample only supports using 10ms SDU
	 * interval. BT_BAP_COEX_INT_MS_10_FAST_2 balances well between sync time (lower
	 * interval is faster) and air time (lower interval require more air time)
	 */
	const struct bt_le_per_adv_param *per_adv_param =
		BT_LE_PER_ADV_PARAM(BT_GAP_MS_TO_PER_ADV_INTERVAL(BT_BAP_COEX_INT_MS_10_FAST_2),
				    BT_GAP_MS_TO_PER_ADV_INTERVAL(BT_BAP_COEX_INT_MS_10_FAST_2),
				    BT_LE_PER_ADV_OPT_NONE);
	int err;

	/* Create a non-connectable advertising set */
	err = bt_le_ext_adv_create(ext_adv_param, NULL, adv);
	if (err != 0) {
		FAIL("Unable to create extended advertising set: %d\n", err);
		return;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(*adv, per_adv_param);
	if (err) {
		FAIL("Failed to set periodic advertising parameters: %d\n", err);
		return;
	}
}

void test_tick(bs_time_t HW_device_time)
{
	if (bst_result != Passed) {
		FAIL("test failed (not passed after %i seconds)\n", WAIT_SECONDS);
	}
}

void test_init(void)
{
	bst_ticker_set_next_tick_absolute(WAIT_TIME);
	bst_result = In_progress;
}

#define SYNC_MSG_SIZE 1
static int32_t dev_cnt;
static uint backchannel_nums[255];
static uint chan_cnt;

static void register_more_cmd_args(void)
{
	static bs_args_struct_t args_struct_toadd[] = {
		{
			.option = "D",
			.name = "number_devices",
			.type = 'i',
			.dest = (void *)&dev_cnt,
			.descript = "Number of devices which will connect in this phy",
			.is_mandatory = true,
		},
		ARG_TABLE_ENDMARKER,
	};

	bs_add_extra_dynargs(args_struct_toadd);
}
NATIVE_TASK(register_more_cmd_args, PRE_BOOT_1, 100);

uint16_t get_dev_cnt(void)
{
	return (uint16_t)dev_cnt;
}

/**
 * @brief Get the channel id based on remote device ID
 *
 * The is effectively a very simple hashing function to generate unique channel IDs from device IDs
 *
 * @param dev 16-bit device ID
 *
 * @return uint 32-bit channel ID
 */
static uint get_chan_num(uint16_t dev)
{
	uint16_t self = (uint16_t)bsim_args_get_global_device_nbr();
	uint channel_id;

	if (self < dev) {
		channel_id = (dev << 16) | self;
	} else {
		channel_id = (self << 16) | dev;
	}

	return channel_id;
}

/**
 * @brief Calculate the sync timeout based on the PA interval
 *
 * Calculates the sync timeout, based on the PA interval and a pre-defined ratio.
 * The return value is in N*10ms, conform the parameter for bt_le_per_adv_sync_create
 *
 * @param pa_interval PA interval
 *
 * @return uint16_t synchronization timeout (N * 10 ms)
 */
uint16_t interval_to_sync_timeout(uint16_t pa_interval)
{
	uint16_t pa_timeout;

	if (pa_interval == BT_BAP_PA_INTERVAL_UNKNOWN) {
		/* Use maximum value to maximize chance of success */
		pa_timeout = BT_GAP_PER_ADV_MAX_TIMEOUT;
	} else {
		uint32_t interval_us;
		uint32_t timeout;

		/* Add retries and convert to unit in 10's of ms */
		interval_us = BT_GAP_PER_ADV_INTERVAL_TO_US(pa_interval);
		timeout = BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(interval_us) *
			  PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO;

		/* Enforce restraints */
		pa_timeout = CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);
	}

	return pa_timeout;
}

/**
 * @brief Set up the backchannels between each pair of device
 *
 * Each pair of devices will get a unique channel
 */
static void setup_backchannels(void)
{
	__ASSERT_NO_MSG(dev_cnt > 0 && dev_cnt < ARRAY_SIZE(backchannel_nums));
	const uint self = bsim_args_get_global_device_nbr();
	uint device_numbers[dev_cnt];
	uint *channels;

	for (int32_t i = 0; i < dev_cnt; i++) {
		if (i != self) { /* skip ourselves*/
			backchannel_nums[chan_cnt] = get_chan_num((uint16_t)i);
			device_numbers[chan_cnt++] = i;
		}
	}

	channels = bs_open_back_channel(self, device_numbers, backchannel_nums, chan_cnt);
	__ASSERT_NO_MSG(channels != NULL);
}
NATIVE_TASK(setup_backchannels, PRE_BOOT_3, 100);

static uint get_chan_id_from_chan_num(uint chan_num)
{
	for (uint i = 0; i < ARRAY_SIZE(backchannel_nums); i++) {
		if (backchannel_nums[i] == chan_num) {
			return i;
		}
	}

	return 0;
}

void backchannel_sync_send(uint dev)
{
	const uint chan_id = get_chan_id_from_chan_num(get_chan_num((uint16_t)dev));
	uint8_t sync_msg[SYNC_MSG_SIZE] = {0};

	printk("Sending sync to %u\n", chan_id);
	bs_bc_send_msg(chan_id, sync_msg, ARRAY_SIZE(sync_msg));
}

void backchannel_sync_send_all(void)
{
	for (int32_t i = 0; i < dev_cnt; i++) {
		const uint self = bsim_args_get_global_device_nbr();

		if (i != self) { /* skip ourselves*/
			backchannel_sync_send(i);
		}
	}
}

void backchannel_sync_wait(uint dev)
{
	const uint chan_id = get_chan_id_from_chan_num(get_chan_num((uint16_t)dev));

	printk("Waiting for sync to %u\n", chan_id);

	while (true) {
		if (bs_bc_is_msg_received(chan_id) > 0) {
			uint8_t sync_msg[SYNC_MSG_SIZE];

			bs_bc_receive_msg(chan_id, sync_msg, ARRAY_SIZE(sync_msg));
			/* We don't really care about the content of the message */
			break;
		}

		k_sleep(K_MSEC(1));
	}
}

void backchannel_sync_wait_all(void)
{
	for (int32_t i = 0; i < dev_cnt; i++) {
		const uint self = bsim_args_get_global_device_nbr();

		if (i != self) { /* skip ourselves*/
			backchannel_sync_wait(i);
		}
	}
}

void backchannel_sync_wait_any(void)
{
	while (true) {
		for (int32_t i = 0; i < dev_cnt; i++) {
			const uint self = bsim_args_get_global_device_nbr();

			if (i != self) { /* skip ourselves*/
				const uint chan_id =
					get_chan_id_from_chan_num(get_chan_num((uint16_t)i));

				if (bs_bc_is_msg_received(chan_id) > 0) {
					uint8_t sync_msg[SYNC_MSG_SIZE];

					bs_bc_receive_msg(chan_id, sync_msg, ARRAY_SIZE(sync_msg));
					/* We don't really care about the content of the message */

					return;
				}
			}
		}

		k_sleep(K_MSEC(100));
	}
}

void backchannel_sync_clear(uint dev)
{
	const uint chan_id = get_chan_id_from_chan_num(get_chan_num((uint16_t)dev));

	while (bs_bc_is_msg_received(chan_id)) {
		uint8_t sync_msg[SYNC_MSG_SIZE];

		bs_bc_receive_msg(chan_id, sync_msg, ARRAY_SIZE(sync_msg));
		/* We don't really care about the content of the message */
	}
}

void backchannel_sync_clear_all(void)
{
	for (int32_t i = 0; i < dev_cnt; i++) {
		const uint self = bsim_args_get_global_device_nbr();

		if (i != self) { /* skip ourselves*/
			backchannel_sync_clear(i);
		}
	}
}
