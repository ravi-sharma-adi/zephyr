/* test_distribute_broadcast_code.c - unit test for distribute broadcast code */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/fff.h>

#include "bluetooth.h"
#include "cap_commander.h"
#include "conn.h"
#include "expects_util.h"
#include "cap_mocks.h"
#include "test_common.h"
#include "test_broadcast_reception.h"
#include "test_distribute_broadcast_code.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_distribute_broadcast_code, CONFIG_BT_CAP_COMMANDER_LOG_LEVEL);

#define FFF_GLOBALS

struct cap_commander_test_distribute_broadcast_code_fixture {
	struct bt_conn conns[CONFIG_BT_MAX_CONN];

	struct bt_bap_bass_subgroup subgroups[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS];
	struct bt_cap_commander_broadcast_reception_start_member_param
		start_member_params[CONFIG_BT_MAX_CONN];
	struct bt_cap_commander_broadcast_reception_start_param start_param;
	struct bt_cap_commander_distribute_broadcast_code_member_param
		broadcast_code_member_params[CONFIG_BT_MAX_CONN];
	struct bt_cap_commander_distribute_broadcast_code_param distribute_broadcast_code_param;
	struct bt_bap_broadcast_assistant_cb broadcast_assistant_cb;
};

/* src_id can not be part of the fixture since it is accessed in the callback function */
static uint8_t src_id[CONFIG_BT_MAX_CONN];

static void cap_commander_broadcast_assistant_recv_state_cb(
	struct bt_conn *conn, int err, const struct bt_bap_scan_delegator_recv_state *state)
{
	uint8_t index;

	index = bt_conn_index(conn);
	src_id[index] = state->src_id;
}

static void cap_commander_test_distribute_broadcast_code_fixture_init(
	struct cap_commander_test_distribute_broadcast_code_fixture *fixture)
{
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(fixture->conns); i++) {
		test_conn_init(&fixture->conns[i]);
		fixture->conns[i].index = i;
	}

	test_start_param_init(fixture);

	fixture->distribute_broadcast_code_param.type = BT_CAP_SET_TYPE_AD_HOC;
	fixture->distribute_broadcast_code_param.param = fixture->broadcast_code_member_params;
	fixture->distribute_broadcast_code_param.count =
		ARRAY_SIZE(fixture->broadcast_code_member_params);
	memcpy(fixture->distribute_broadcast_code_param.broadcast_code, BROADCAST_CODE,
	       sizeof(BROADCAST_CODE));
	for (size_t i = 0; i < ARRAY_SIZE(fixture->broadcast_code_member_params); i++) {
		fixture->broadcast_code_member_params[i].member.member = &fixture->conns[i];
		fixture->broadcast_code_member_params[i].src_id = 0;
	}

	fixture->broadcast_assistant_cb.recv_state =
		cap_commander_broadcast_assistant_recv_state_cb;
	err = bt_bap_broadcast_assistant_register_cb(&fixture->broadcast_assistant_cb);
	zassert_equal(0, err, "Failed registering broadcast assistant callback functions %d", err);
}

static void *cap_commander_test_distribute_broadcast_code_setup(void)
{
	struct cap_commander_test_distribute_broadcast_code_fixture *fixture;

	fixture = malloc(sizeof(*fixture));
	zassert_not_null(fixture);

	return fixture;
}

static void cap_commander_test_distribute_broadcast_code_before(void *f)
{
	int err;
	struct cap_commander_test_distribute_broadcast_code_fixture *fixture = f;

	memset(f, 0, sizeof(struct cap_commander_test_distribute_broadcast_code_fixture));
	cap_commander_test_distribute_broadcast_code_fixture_init(fixture);

	for (size_t i = 0; i < ARRAY_SIZE(fixture->conns); i++) {
		err = bt_cap_commander_discover(&fixture->conns[i]);
		zassert_equal(0, err, "Unexpected return value %d", err);
	}
}

static void cap_commander_test_distribute_broadcast_code_after(void *f)
{
	struct cap_commander_test_distribute_broadcast_code_fixture *fixture = f;

	bt_cap_commander_unregister_cb(&mock_cap_commander_cb);
	bt_bap_broadcast_assistant_unregister_cb(&fixture->broadcast_assistant_cb);

	/* We need to cleanup since the CAP commander remembers state */
	bt_cap_commander_cancel();

	for (size_t i = 0; i < ARRAY_SIZE(fixture->conns); i++) {
		mock_bt_conn_disconnected(&fixture->conns[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static void cap_commander_test_distribute_broadcast_code_teardown(void *f)
{
	free(f);
}

static void test_distribute_broadcast_code(
	struct bt_cap_commander_distribute_broadcast_code_param *distribute_broadcast_code_param)
{
	int err;

	err = bt_cap_commander_distribute_broadcast_code(distribute_broadcast_code_param);
	zassert_equal(0, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 1,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
	zassert_equal_ptr(NULL,
			  mock_cap_commander_distribute_broadcast_code_cb_fake.arg0_history[0]);
	zassert_equal(0, mock_cap_commander_distribute_broadcast_code_cb_fake.arg1_history[0]);
}

ZTEST_SUITE(cap_commander_test_distribute_broadcast_code, NULL,
	    cap_commander_test_distribute_broadcast_code_setup,
	    cap_commander_test_distribute_broadcast_code_before,
	    cap_commander_test_distribute_broadcast_code_after,
	    cap_commander_test_distribute_broadcast_code_teardown);

ZTEST_F(cap_commander_test_distribute_broadcast_code, test_commander_distribute_broadcast_code)
{
	int err;

	err = bt_cap_commander_register_cb(&mock_cap_commander_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	/* we perform a broadcast_reception_start to ensure we get a valid src_id */
	test_broadcast_reception_start(&fixture->start_param);

	for (size_t i = 0U; i < CONFIG_BT_MAX_CONN; i++) {
		fixture->distribute_broadcast_code_param.param[i].src_id = src_id[i];
	}

	test_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_reception_distribute_broadcast_code_double)
{
	int err;

	err = bt_cap_commander_register_cb(&mock_cap_commander_cb);
	zassert_equal(0, err, "Unexpected return value %d", err);

	test_broadcast_reception_start(&fixture->start_param);

	for (size_t i = 0U; i < CONFIG_BT_MAX_CONN; i++) {
		fixture->distribute_broadcast_code_param.param[i].src_id = src_id[i];
	}

	test_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);

	/*
	 * We can not use test_distribute_broadcast_code because of the check on how often the
	 * callback function is called
	 */
	err = bt_cap_commander_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
	zassert_equal(0, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 2,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_reception_distribute_broadcast_code_param_null)
{
	int err;

	err = bt_cap_commander_distribute_broadcast_code(NULL);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 0,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_distribute_broadcast_code_param_zero_count)
{
	int err;

	fixture->distribute_broadcast_code_param.count = 0;

	err = bt_cap_commander_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 0,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_distribute_broadcast_code_param_high_count)
{
	int err;

	fixture->distribute_broadcast_code_param.count = CONFIG_BT_MAX_CONN + 1;

	err = bt_cap_commander_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.broadcast_distribute_broadcast_code", 0,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_distribute_broadcast_code_inval_param_null_param)
{
	int err;

	fixture->distribute_broadcast_code_param.type = BT_CAP_SET_TYPE_AD_HOC;
	fixture->distribute_broadcast_code_param.param = NULL;
	fixture->distribute_broadcast_code_param.count = ARRAY_SIZE(fixture->conns);

	err = bt_cap_commander_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 0,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_distribute_broadcast_code_inval_null_member)
{
	int err;

	fixture->distribute_broadcast_code_param.param[0].member.member = NULL;

	err = bt_cap_commander_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 0,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}

ZTEST_F(cap_commander_test_distribute_broadcast_code,
	test_commander_distribute_broadcast_code_inval_missing_cas)
{
	int err;

	fixture->distribute_broadcast_code_param.type = BT_CAP_SET_TYPE_CSIP;

	err = bt_cap_commander_distribute_broadcast_code(&fixture->distribute_broadcast_code_param);
	zassert_equal(-EINVAL, err, "Unexpected return value %d", err);

	zexpect_call_count("bt_cap_commander_cb.distribute_broadcast_code", 0,
			   mock_cap_commander_distribute_broadcast_code_cb_fake.call_count);
}
