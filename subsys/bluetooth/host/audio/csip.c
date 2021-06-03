/* Bluetooth CSIS - Coordinated Set Identification Client
 *
 * Copyright (c) 2020 Bose Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CSIP should be used in the following way
 *  1) Find and connect to a set device
 *  2) Do discovery
 *  3) read values (always SIRK, size, lock and rank if possible)
 *  4) Discover other set members if applicable
 *  5) Connect and bond with each set member
 *  6) Do discovery of each member
 *  7) Read rank for each set member
 *  8) Lock all members based on rank if possible
 *  9) Do whatever is needed during lock
 * 10) Unlock all members
 */

#include <zephyr.h>
#include <zephyr/types.h>

#include <device.h>
#include <init.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/buf.h>
#include <sys/byteorder.h>
#include "csip.h"
#include "csis_crypto.h"
#include "../conn_internal.h"
#include "../keys.h"
#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_CSIP)
#define LOG_MODULE_NAME bt_csip
#include "common/log.h"

#define FIRST_HANDLE                    0x0001
#define LAST_HANDLE                     0xFFFF
#define CSIP_DISCOVER_TIMER_VALUE       K_SECONDS(10)

struct csis_instance_t {
	uint8_t rank;
	uint8_t set_lock;

	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t set_sirk_handle;
	uint16_t set_size_handle;
	uint16_t set_lock_handle;
	uint16_t rank_handle;

	uint8_t idx;
	struct bt_gatt_subscribe_params sirk_sub_params;
	struct bt_gatt_discover_params sirk_sub_disc_params;
	struct bt_gatt_subscribe_params size_sub_params;
	struct bt_gatt_discover_params size_sub_disc_params;
	struct bt_gatt_subscribe_params lock_sub_params;
	struct bt_gatt_discover_params lock_sub_disc_params;
};

struct bt_csis_server {
	bool locked_by_us;
	uint8_t inst_count;
	struct csis_instance_t csis_insts[CONFIG_BT_CSIP_MAX_CSIS_INSTANCES];
	struct bt_csip_set_member set_member;
};

static uint8_t gatt_write_buf[1];
static struct bt_gatt_write_params write_params;
static struct bt_gatt_read_params read_params;
static bool subscribe_all;
static struct bt_gatt_discover_params discover_params;
static struct csis_instance_t *cur_inst;
static struct bt_csip_set_t cur_set;
static bool busy;
static struct k_work_delayable discover_members_timer;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);

static struct bt_csip_cb_t *csip_cbs;
static bt_csip_discover_cb_t init_cb;
static bt_csip_discover_sets_cb_t discover_sets_cb;
static uint8_t members_found;
static struct bt_csis_server servers[CONFIG_BT_MAX_CONN];
static struct bt_csip_set_member set_member_addrs[CONFIG_BT_MAX_CONN];

static int read_set_sirk(struct bt_conn *conn, uint8_t inst_idx);
static int csip_read_set_size(struct bt_conn *conn, uint8_t inst_idx,
			      bt_gatt_read_func_t cb);

static uint8_t csip_discover_sets_read_set_sirk_cb(
	struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
	const void *data, uint16_t length);
static void csip_lock_set_write_lock_cb(struct bt_conn *conn, uint8_t err,
	struct bt_gatt_write_params *params);
static void csip_release_set_write_lock_cb(struct bt_conn *conn, uint8_t err,
	struct bt_gatt_write_params *params);
static void csip_lock_set_init_cb(struct bt_conn *conn, int err,
				  uint8_t set_count);
static void discover_sets_resume(struct bt_conn *conn, uint16_t sirk_handle,
				 uint16_t size_handle, uint16_t rank_handle);

static struct csis_instance_t *lookup_instance_by_handle(struct bt_conn *conn,
							 uint16_t handle)
{
	uint8_t conn_index;
	struct bt_csis_server *server_inst;

	__ASSERT(conn, "NULL conn");
	__ASSERT(handle > 0, "Handle cannot be 0");

	conn_index = bt_conn_index(conn);
	server_inst = &servers[conn_index];

	for (int i = 0; i < ARRAY_SIZE(server_inst->csis_insts); i++) {
		if (server_inst->csis_insts[i].start_handle <= handle &&
		    server_inst->csis_insts[i].end_handle >= handle) {
			return &server_inst->csis_insts[i];
		}
	}

	return NULL;
}

static struct csis_instance_t *lookup_instance_by_index(struct bt_conn *conn,
							uint8_t idx)
{
	uint8_t conn_index;
	struct bt_csis_server *server_inst;

	__ASSERT(conn, "NULL conn");
	__ASSERT(idx < CONFIG_BT_CSIP_MAX_CSIS_INSTANCES,
		 "Index shall be less than maximum number of instances %u (was %u)",
		 CONFIG_BT_CSIP_MAX_CSIS_INSTANCES, idx);

	conn_index = bt_conn_index(conn);
	server_inst = &servers[conn_index];
	return &server_inst->csis_insts[idx];
}

static int sirk_decrypt(struct bt_conn *conn,
			const struct bt_csip_set_sirk_t *enc_sirk,
			struct bt_csip_set_sirk_t *out_sirk)
{
	int err;
	uint8_t *k;

	if (IS_ENABLED(CONFIG_BT_CSIP_TEST_SAMPLE_DATA)) {
		/* test_k is from the sample data from A.2 in the CSIS spec */
		static uint8_t test_k[] = {0x67, 0x6e, 0x1b, 0x9b,
					   0xd4, 0x48, 0x69, 0x6f,
					   0x06, 0x1e, 0xc6, 0x22,
					   0x3c, 0xe5, 0xce, 0xd9};
		static bool swapped;

		BT_DBG("Decrypting with sample data K");

		if (!swapped && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) {
			/* Swap test_k to little endian */
			sys_mem_swap(test_k, 16);
			swapped = true;
		}
		k = test_k;
	} else {
		k = conn->le.keys->ltk.val;
	}

	err = bt_csis_sdf(k, enc_sirk->value, out_sirk->value);

	return err;
}

static int8_t lookup_index_by_sirk(struct bt_conn *conn,
				   struct bt_csip_set_sirk_t *sirk)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];

	for (int i = 0; i < CONFIG_BT_CSIP_MAX_CSIS_INSTANCES; i++) {
		struct bt_csip_set_sirk_t *set_sirk = &server->set_member.sets[i].set_sirk;

		if (memcmp(sirk->value, set_sirk->value, sizeof(set_sirk->value)) == 0) {
			return i;
		}
	}

	return -1;
}

static uint8_t sirk_notify_func(struct bt_conn *conn,
				struct bt_gatt_subscribe_params *params,
				const void *data, uint16_t length)
{
	uint16_t handle = params->value_handle;
	struct csis_instance_t *csis_inst;

	if (!data) {
		BT_DBG("[UNSUBSCRIBED] %u", params->value_handle);
		params->value_handle = 0U;

		return BT_GATT_ITER_STOP;
	}

	csis_inst = lookup_instance_by_handle(conn, handle);

	if (csis_inst) {
		BT_DBG("Instance %u", csis_inst->idx);
		if (length == sizeof(cur_set.set_sirk)) {
			struct bt_csip_set_sirk_t *sirk =
				(struct bt_csip_set_sirk_t *)data;

			BT_DBG("Set SIRK %sencrypted",
			       sirk->type == BT_CSIP_SIRK_TYPE_PLAIN
				? "not " : "");

			/* Assuming not connected to other set devices */
			if (sirk->type == BT_CSIP_SIRK_TYPE_ENCRYPTED) {
				if (IS_ENABLED(
					CONFIG_BT_CSIP_ENC_SIRK_SUPPORT)) {
					int err;

					BT_HEXDUMP_DBG(sirk->value,
						       sizeof(*sirk),
						       "Encrypted Set SIRK");
					err = sirk_decrypt(conn, sirk,
							   &cur_set.set_sirk);
					if (err) {
						BT_ERR("Could not decrypt "
						       "SIRK %d", err);
					}
				} else {
					BT_DBG("Encrypted SIRK not supported");
					return BT_GATT_ITER_CONTINUE;
				}
			} else {
				memcpy(&cur_set.set_sirk, data, length);
			}

			BT_HEXDUMP_DBG(cur_set.set_sirk.value,
				       sizeof(cur_set.set_sirk.value),
				       "Set SIRK");
		} else {
			BT_DBG("Invalid length %u", length);
		}
	} else {
		BT_DBG("Notification/Indication on unknown CSIS inst");
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t size_notify_func(struct bt_conn *conn,
				struct bt_gatt_subscribe_params *params,
				const void *data, uint16_t length)
{
	uint8_t size;
	uint16_t handle = params->value_handle;
	struct csis_instance_t *csis_inst;

	if (!data) {
		BT_DBG("[UNSUBSCRIBED] %u", params->value_handle);
		params->value_handle = 0U;

		return BT_GATT_ITER_STOP;
	}

	csis_inst = lookup_instance_by_handle(conn, handle);

	if (csis_inst) {
		if (length == sizeof(cur_set.set_size)) {
			memcpy(&size, data, length);
			BT_DBG("Set size updated from %u to %u",
			       cur_set.set_size, size);
			cur_set.set_size = size;
		} else {
			BT_DBG("Invalid length %u", length);
		}

	} else {
		BT_DBG("Notification/Indication on unknown CSIS inst");
	}
	BT_HEXDUMP_DBG(data, length, "Value");

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t lock_notify_func(struct bt_conn *conn,
				struct bt_gatt_subscribe_params *params,
				const void *data, uint16_t length)
{
	uint8_t value;
	uint16_t handle = params->value_handle;
	struct csis_instance_t *csis_inst;

	if (!data) {
		BT_DBG("[UNSUBSCRIBED] %u", params->value_handle);
		params->value_handle = 0U;

		return BT_GATT_ITER_STOP;
	}

	csis_inst = lookup_instance_by_handle(conn, handle);

	if (csis_inst) {
		if (length == sizeof(csis_inst->set_lock)) {
			bool locked;

			memcpy(&value, data, length);
			if (value != BT_CSIP_RELEASE_VALUE &&
			    value != BT_CSIP_LOCK_VALUE) {
				BT_DBG("Invalid value %u", value);
				return BT_GATT_ITER_STOP;
			}

			memcpy(&csis_inst->set_lock, data, length);

			locked = csis_inst->set_lock == BT_CSIP_LOCK_VALUE;
			BT_DBG("Instance %u lock was %s",
			       csis_inst->idx, locked ? "locked" : "released");
			if (csip_cbs && csip_cbs->lock_changed) {
				csip_cbs->lock_changed(conn, &cur_set, locked);
			}
		} else {
			BT_DBG("Invalid length %u", length);
		}
	} else {
		BT_DBG("Notification/Indication on unknown CSIS inst");
	}
	BT_HEXDUMP_DBG(data, length, "Value");

	return BT_GATT_ITER_CONTINUE;
}

static int csip_write_set_lock(struct bt_conn *conn, uint8_t inst_idx,
			       bool lock, bt_gatt_write_func_t cb)
{

	if (inst_idx >= CONFIG_BT_CSIP_MAX_CSIS_INSTANCES) {
		return -EINVAL;
	} else if (cur_inst) {
		if (cur_inst != lookup_instance_by_index(conn, inst_idx)) {
			return -EBUSY;
		}
	} else {
		cur_inst = lookup_instance_by_index(conn, inst_idx);
		if (!cur_inst) {
			BT_DBG("Inst not found");
			return -EINVAL;
		}
	}

	if (!cur_inst->rank_handle) {
		BT_DBG("Handle not set");
		cur_inst = NULL;
		return -EINVAL;
	}

	/* Write to call control point */
	gatt_write_buf[0] = lock ? BT_CSIP_LOCK_VALUE : BT_CSIP_RELEASE_VALUE;
	write_params.data = gatt_write_buf;
	write_params.length = sizeof(lock);
	write_params.func = cb;
	write_params.handle = cur_inst->set_lock_handle;

	return bt_gatt_write(conn, &write_params);
}

static int read_set_sirk(struct bt_conn *conn, uint8_t inst_idx)
{
	if (inst_idx >= CONFIG_BT_CSIP_MAX_CSIS_INSTANCES) {
		return -EINVAL;
	} else if (cur_inst) {
		if (cur_inst != lookup_instance_by_index(conn, inst_idx)) {
			return -EBUSY;
		}
	} else {
		cur_inst = lookup_instance_by_index(conn, inst_idx);
		if (!cur_inst) {
			BT_DBG("Inst not found");
			return -EINVAL;
		}
	}

	if (!cur_inst->set_sirk_handle) {
		BT_DBG("Handle not set");
		cur_inst = NULL;
		return -EINVAL;
	}

	read_params.func = csip_discover_sets_read_set_sirk_cb;
	read_params.handle_count = 1;
	read_params.single.handle = cur_inst->set_sirk_handle;
	read_params.single.offset = 0U;

	return bt_gatt_read(conn, &read_params);
}

static int csip_read_set_size(struct bt_conn *conn, uint8_t inst_idx,
			      bt_gatt_read_func_t cb)
{
	if (inst_idx >= CONFIG_BT_CSIP_MAX_CSIS_INSTANCES) {
		return -EINVAL;
	} else if (cur_inst) {
		if (cur_inst != lookup_instance_by_index(conn, inst_idx)) {
			return -EBUSY;
		}
	} else {
		cur_inst = lookup_instance_by_index(conn, inst_idx);
		if (!cur_inst) {
			BT_DBG("Inst not found");
			return -EINVAL;
		}
	}

	if (!cur_inst->set_size_handle) {
		BT_DBG("Handle not set");
		cur_inst = NULL;
		return -EINVAL;
	}

	read_params.func = cb;
	read_params.handle_count = 1;
	read_params.single.handle = cur_inst->set_size_handle;
	read_params.single.offset = 0U;

	return bt_gatt_read(conn, &read_params);
}

static int csip_read_rank(struct bt_conn *conn, uint8_t inst_idx,
			  bt_gatt_read_func_t cb)
{
	if (inst_idx >= CONFIG_BT_CSIP_MAX_CSIS_INSTANCES) {
		return -EINVAL;
	} else if (cur_inst) {
		if (cur_inst != lookup_instance_by_index(conn, inst_idx)) {
			return -EBUSY;
		}
	} else {
		cur_inst = lookup_instance_by_index(conn, inst_idx);
		if (!cur_inst) {
			BT_DBG("Inst not found");
			return -EINVAL;
		}
	}

	if (!cur_inst->rank_handle) {
		BT_DBG("Handle not set");
		cur_inst = NULL;
		return -EINVAL;
	}

	read_params.func = cb;
	read_params.handle_count = 1;
	read_params.single.handle = cur_inst->rank_handle;
	read_params.single.offset = 0U;

	return bt_gatt_read(conn, &read_params);
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	struct bt_gatt_chrc *chrc;
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];
	struct bt_gatt_subscribe_params *sub_params = NULL;
	void *notify_handler = NULL;

	if (!attr) {
		BT_DBG("Setup complete for %u / %u",
		       cur_inst->idx + 1, server->inst_count);
		(void)memset(params, 0, sizeof(*params));

		if ((cur_inst->idx + 1) < server->inst_count) {
			int err;

			cur_inst = &server->csis_insts[cur_inst->idx + 1];
			discover_params.uuid = NULL;
			discover_params.start_handle =
				cur_inst->start_handle;
			discover_params.end_handle =
				cur_inst->end_handle;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
			discover_params.func = discover_func;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				BT_DBG("Discover failed (err %d)", err);
				cur_inst = NULL;
				busy = false;
				if (init_cb) {
					init_cb(conn, err, server->inst_count);
				}
			}

		} else {
			cur_inst = NULL;
			busy = false;
			if (init_cb) {
				init_cb(conn, 0, server->inst_count);
			}
		}
		return BT_GATT_ITER_STOP;
	}

	BT_DBG("[ATTRIBUTE] handle 0x%04X", attr->handle);

	if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC &&
	    server->inst_count) {
		chrc = (struct bt_gatt_chrc *)attr->user_data;
		if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CSIS_SET_SIRK)) {
			BT_DBG("Set SIRK");
			cur_inst->set_sirk_handle = chrc->value_handle;
			sub_params = &cur_inst->sirk_sub_params;
			sub_params->disc_params = &cur_inst->sirk_sub_disc_params;
			notify_handler = sirk_notify_func;
		} else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CSIS_SET_SIZE)) {
			BT_DBG("Set size");
			cur_inst->set_size_handle = chrc->value_handle;
			sub_params = &cur_inst->size_sub_params;
			sub_params->disc_params = &cur_inst->size_sub_disc_params;
			notify_handler = size_notify_func;
		} else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CSIS_SET_LOCK)) {
			BT_DBG("Set lock");
			cur_inst->set_lock_handle = chrc->value_handle;
			sub_params = &cur_inst->lock_sub_params;
			sub_params->disc_params = &cur_inst->lock_sub_disc_params;
			notify_handler = lock_notify_func;
		} else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CSIS_RANK)) {
			BT_DBG("Set rank");
			cur_inst->rank_handle = chrc->value_handle;
		}

		if (subscribe_all && sub_params && notify_handler) {
			sub_params->value = 0;
			if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
				sub_params->value = BT_GATT_CCC_NOTIFY;
			} else if (chrc->properties & BT_GATT_CHRC_INDICATE) {
				sub_params->value = BT_GATT_CCC_INDICATE;
			}

			if (sub_params->value) {
				/* With ccc_handle == 0 it will use auto discovery */
				sub_params->ccc_handle = 0;
				sub_params->end_handle = cur_inst->end_handle;
				sub_params->value_handle = chrc->value_handle;
				sub_params->notify = notify_handler;
				bt_gatt_subscribe(conn, sub_params);
			}
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t primary_discover_func(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	struct bt_gatt_service_val *prim_service;
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];

	if (!attr || server->inst_count == CONFIG_BT_CSIP_MAX_CSIS_INSTANCES) {
		BT_DBG("Discover complete, found %u instances",
		       server->inst_count);
		(void)memset(params, 0, sizeof(*params));

		if (server->inst_count) {
			int err;

			cur_inst = &server->csis_insts[0];
			discover_params.uuid = NULL;
			discover_params.start_handle =
				cur_inst->start_handle;
			discover_params.end_handle = cur_inst->end_handle;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
			discover_params.func = discover_func;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				BT_DBG("Discover failed (err %d)", err);
				busy = false;
				cur_inst = NULL;
				if (init_cb) {
					init_cb(conn, err, server->inst_count);
				}
			}
		} else {
			busy = false;
			cur_inst = NULL;
			if (init_cb) {
				init_cb(conn, 0, 0);
			}
		}
		return BT_GATT_ITER_STOP;
	}

	BT_DBG("[ATTRIBUTE] handle 0x%04X", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		prim_service = (struct bt_gatt_service_val *)attr->user_data;
		discover_params.start_handle = attr->handle + 1;

		cur_inst = &server->csis_insts[server->inst_count];
		cur_inst->idx = server->inst_count;
		cur_inst->start_handle = attr->handle + 1;
		cur_inst->end_handle = prim_service->end_handle;
		server->inst_count++;
	}

	return BT_GATT_ITER_CONTINUE;
}

bool bt_csip_is_set_member(uint8_t set_sirk[BT_CSIP_SET_SIRK_SIZE],
			   struct bt_data *data)
{
	if (data->type == BT_DATA_CSIS_RSI &&
	    data->data_len == BT_CSIS_PSRI_SIZE) {
		uint8_t err;

		uint32_t hash = sys_get_le24(data->data);
		uint32_t prand = sys_get_le24(data->data + 3);
		uint32_t calculated_hash;

		BT_DBG("hash: 0x%06x, prand 0x%06x", hash, prand);
		err = bt_csis_sih(set_sirk, prand, &calculated_hash);
		if (err) {
			return false;
		}

		calculated_hash &= 0xffffff;

		BT_DBG("calculated_hash: 0x%06x, hash 0x%06x",
		       calculated_hash, hash);

		return calculated_hash == hash;
	}

	return false;
}

static bool is_discovered(const bt_addr_le_t *addr)
{
	for (int i = 0; i < members_found; i++) {
		if (!bt_addr_le_cmp(addr, &set_member_addrs[i].addr)) {
			return true;
		}
	}
	return false;
}

static bool csis_found(struct bt_data *data, void *user_data)
{
	if (bt_csip_is_set_member(cur_set.set_sirk.value, data)) {
		bt_addr_le_t *addr = user_data;
		char addr_str[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
		BT_DBG("Found CSIS advertiser with address %s",
		       log_strdup(addr_str));

		if (is_discovered(addr)) {
			BT_DBG("Set member already found");
			/* Stop parsing */
			return false;
		}

		bt_addr_le_copy(&set_member_addrs[members_found].addr, addr);

		members_found++;
		BT_DBG("Found member (%u / %u)",
			members_found, cur_set.set_size);

		if (members_found == cur_set.set_size) {
			(void)k_work_cancel_delayable(&discover_members_timer);
			bt_le_scan_stop();
			busy = false;
			cur_inst = NULL;
			if (csip_cbs && csip_cbs->members) {
				csip_cbs->members(0, cur_set.set_size,
						  members_found);
			}
		}
		/* Stop parsing */
		return false;
	}
	/* Continue parsing */
	return true;
}

static int init_discovery(struct bt_csis_server *server, bool subscribe,
			  bt_csip_discover_cb_t cb)
{
	init_cb = cb;
	server->inst_count = 0;
	/* Discover CSIS on peer, setup handles and notify */
	subscribe_all = subscribe;
	(void)memset(&discover_params, 0, sizeof(discover_params));
	memcpy(&uuid, BT_UUID_CSIS, sizeof(uuid));
	discover_params.func = primary_discover_func;
	discover_params.uuid = &uuid.uuid;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;
	discover_params.start_handle = FIRST_HANDLE;
	discover_params.end_handle = LAST_HANDLE;

	return bt_gatt_discover(server->set_member.conn, &discover_params);
}

static uint8_t csip_discover_sets_read_rank_cb(
	struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
	const void *data, uint16_t length)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];
	int cb_err = err;

	__ASSERT(cur_inst, "cur_inst must not be NULL");

	busy = false;


	if (err) {
		BT_DBG("err: 0x%02X", err);
	} else if (data) {
		BT_HEXDUMP_DBG(data, length, "Data read");

		if (length == 1) {
			memcpy(&server->csis_insts[cur_inst->idx].rank,
			       data, length);
			BT_DBG("%u", server->csis_insts[cur_inst->idx].rank);
		} else {
			BT_DBG("Invalid length, continuing to next member");
		}

		discover_sets_resume(conn, 0, 0, 0);
	}

	if (cb_err) {
		if (discover_sets_cb) {
			discover_sets_cb(conn, cb_err, server->inst_count,
					 server->set_member.sets);
		}
	}

	return BT_GATT_ITER_STOP;
}

static uint8_t csip_discover_sets_read_set_size_cb(
	struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
	const void *data, uint16_t length)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];
	int cb_err = err;

	__ASSERT(cur_inst, "cur_inst must not be NULL");

	busy = false;

	if (err) {
		BT_DBG("err: 0x%02X", err);
	} else if (data) {
		BT_HEXDUMP_DBG(data, length, "Data read");

		if (length == 1) {
			memcpy(&server->set_member.sets[cur_inst->idx].set_size,
			       data, length);
			BT_DBG("%u", server->set_member.sets[cur_inst->idx].set_size);
		} else {
			BT_DBG("Invalid length");
		}

		discover_sets_resume(conn, 0, 0, cur_inst->rank_handle);
	}

	if (cb_err) {
		if (discover_sets_cb) {
			discover_sets_cb(conn, cb_err, server->inst_count,
					 server->set_member.sets);
		}
	}

	return BT_GATT_ITER_STOP;
}

static int parse_sirk(struct bt_csip_set_member *member, const void *data,
		      uint16_t length)
{
	struct bt_csip_set_sirk_t *set_sirk;

	set_sirk = &member->sets[cur_inst->idx].set_sirk;

	if (length == sizeof(*set_sirk)) {
		struct bt_csip_set_sirk_t *sirk =
			(struct bt_csip_set_sirk_t *)data;

		BT_DBG("Set SIRK %sencrypted",
		       sirk->type == BT_CSIP_SIRK_TYPE_PLAIN ? "not " : "");
		/* Assuming not connected to other set devices */
		if (sirk->type == BT_CSIP_SIRK_TYPE_ENCRYPTED) {
			if (IS_ENABLED(CONFIG_BT_CSIP_ENC_SIRK_SUPPORT)) {
				int err;

				BT_HEXDUMP_DBG(sirk->value, sizeof(sirk->value),
					       "Encrypted Set SIRK");
				err = sirk_decrypt(member->conn, sirk,
						   set_sirk);
				if (err) {
					BT_ERR("Could not decrypt "
						"SIRK %d", err);
					return err;
				}
			} else {
				BT_WARN("Encrypted SIRK not supported");
				set_sirk = NULL;
				return BT_ATT_ERR_INSUFFICIENT_ENCRYPTION;
			}
		} else {
			memcpy(set_sirk, data, length);
		}

		if (set_sirk) {
			BT_HEXDUMP_DBG(set_sirk->value, sizeof(set_sirk->value),
				       "Set SIRK");
		}
	} else {
		BT_DBG("Invalid length");
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	}

	return 0;
}

static uint8_t csip_discover_sets_read_set_sirk_cb(
	struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
	const void *data, uint16_t length)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];
	int cb_err = err;

	__ASSERT(cur_inst, "cur_inst must not be NULL");

	busy = false;

	if (err) {
		BT_DBG("err: 0x%02X", err);
	} else if (data) {
		BT_HEXDUMP_DBG(data, length, "Data read");

		cb_err = parse_sirk(&server->set_member, data, length);

		if (cb_err) {
			BT_DBG("Could not parse SIRK: %d", cb_err);
		} else {
			discover_sets_resume(conn, 0, cur_inst->set_size_handle,
					     cur_inst->rank_handle);
		}
	}

	if (cb_err) {
		if (discover_sets_cb) {
			discover_sets_cb(conn, cb_err, server->inst_count,
					 server->set_member.sets);
		}
	}

	return BT_GATT_ITER_STOP;
}

/**
 * @brief Reads the (next) characteristics for the set discovery procedure
 *
 * It skips all handles that are 0.
 *
 * @param conn        Connection to a CSIS device.
 * @param sirk_handle 0, or the handle for the SIRK characteristic.
 * @param size_handle 0, or the handle for the size characteristic.
 * @param rank_handle 0, or the handle for the rank characteristic.
 */
static void discover_sets_resume(struct bt_conn *conn, uint16_t sirk_handle,
				 uint16_t size_handle, uint16_t rank_handle)
{
	int cb_err = 0;
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];

	if (size_handle) {
		cb_err = csip_read_set_size(
				conn, cur_inst->idx,
				csip_discover_sets_read_set_size_cb);
		if (cb_err) {
			BT_DBG("Could not read set size: %d", cb_err);
		}
	} else if (rank_handle) {
		cb_err = csip_read_rank(
				conn, cur_inst->idx,
				csip_discover_sets_read_rank_cb);
		if (cb_err) {
			BT_DBG("Could not read set rank: %d", cb_err);
		}
	} else {
		uint8_t next_idx = cur_inst->idx + 1;

		cur_inst = NULL;
		if (next_idx < server->inst_count) {
			/* Read next */
			cb_err = read_set_sirk(conn, next_idx);
		} else if (discover_sets_cb) {
			discover_sets_cb(conn, 0, server->inst_count,
						server->set_member.sets);
		}

		return;
	}

	if (cb_err) {
		if (discover_sets_cb) {
			discover_sets_cb(conn, cb_err, server->inst_count,
					 server->set_member.sets);
		}
	} else {
		busy = true;
	}
}

static void csip_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		BT_DBG("Failed to connect to %s (%u)", log_strdup(addr),
			err);
		return;
	}

	BT_DBG("Connected to %s", log_strdup(addr));

	err = init_discovery(server, true, csip_lock_set_init_cb);

	if (err) {
		if (csip_cbs && csip_cbs->lock_set) {
			csip_cbs->lock_set(err);
		}
	}
}

static void csip_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];

	if (server->set_member.conn == conn) {
		char addr[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		BT_DBG("Disconnected: %s (reason %u)", log_strdup(addr),
		       reason);
	}
}

static struct bt_conn_cb csip_conn_callbacks = {
	.connected = csip_connected,
	.disconnected = csip_disconnected,
};

static void csip_scan_recv(const struct bt_le_scan_recv_info *info,
			   struct net_buf_simple *ad)
{
	/* We're only interested in connectable events */
	if (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) {
		bt_data_parse(ad, csis_found, (void *)info->addr);

	}
}

static struct bt_le_scan_cb csip_scan_callbacks = {
	.recv = csip_scan_recv
};

static void csip_write_lowest_rank(void)
{
	uint8_t min_rank = 0xFF;
	uint8_t min_idx;
	struct bt_conn *min_conn = NULL;

	cur_inst = NULL;

	for (int i = 0; i < cur_set.set_size; i++) {
		int8_t cur_idx = lookup_index_by_sirk(servers[i].set_member.conn,
						      &cur_set.set_sirk);

		if (cur_idx < 0) {
			continue;
		}
		if (servers[i].csis_insts[cur_idx].rank < min_rank &&
		    !servers[i].locked_by_us) {
			min_rank = servers[i].csis_insts[cur_idx].rank;
			min_idx = cur_idx;
			min_conn = servers[i].set_member.conn;
		}
	}

	if (min_conn) {
		int err;

		BT_DBG("Locking member with rank %u", min_rank);
		err = csip_write_set_lock(min_conn, min_idx, true,
					  csip_lock_set_write_lock_cb);
		if (err) {
			if (csip_cbs && csip_cbs->lock_set) {
				csip_cbs->lock_set(err);
			}
		}
	} else {
		busy = false;
		cur_inst = NULL;
		if (csip_cbs && csip_cbs->lock_set) {
			csip_cbs->lock_set(0);
		}
	}
}

static void csip_release_highest_rank(void)
{
	uint8_t max_rank = 0;
	uint8_t max_idx;
	struct bt_conn *max_conn = NULL;

	cur_inst = NULL;

	for (int i = 0; i < cur_set.set_size; i++) {
		int8_t cur_idx = lookup_index_by_sirk(servers[i].set_member.conn,
						      &cur_set.set_sirk);

		if (cur_idx < 0) {
			continue;
		}
		if (servers[i].csis_insts[cur_idx].rank > max_rank &&
		    servers[i].locked_by_us) {
			max_rank = servers[i].csis_insts[cur_idx].rank;
			max_idx = cur_idx;
			max_conn = servers[i].set_member.conn;
		}
	}

	if (max_conn) {
		BT_DBG("Releasing member with rank %u", max_rank);
		csip_write_set_lock(max_conn, max_idx, false,
				    csip_release_set_write_lock_cb);
	} else {
		busy = false;
		cur_inst = NULL;
		if (csip_cbs && csip_cbs->release_set) {
			csip_cbs->release_set(0);
		}
	}
}

static void csip_release_set_write_lock_cb(struct bt_conn *conn, uint8_t err,
					   struct bt_gatt_write_params *params)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];

	if (err) {
		busy = false;
		cur_inst = NULL;
		if (csip_cbs && csip_cbs->release_set) {
			csip_cbs->release_set(err);
		}
		return;
	}

	server->locked_by_us = false;

	csip_release_highest_rank();
}

static void csip_lock_set_write_lock_cb(struct bt_conn *conn, uint8_t err,
					struct bt_gatt_write_params *params)
{
	struct bt_csis_server *server = &servers[bt_conn_index(conn)];

	if (err) {
		busy = false;
		cur_inst = NULL;
		BT_WARN("Could not lock set (%d)", err);
		if (csip_cbs && csip_cbs->lock_set) {
			csip_cbs->lock_set(err);
		}
		return;
	}

	server->locked_by_us = true;

	csip_write_lowest_rank();
}

static bool csip_set_is_locked(void)
{
	for (int i = 0; i < cur_set.set_size; i++) {
		int8_t cur_idx = lookup_index_by_sirk(servers[i].set_member.conn,
						      &cur_set.set_sirk);

		if (servers[i].csis_insts[cur_idx].set_lock ==
			BT_CSIP_LOCK_VALUE) {
			return true;
		}
	}
	return false;
}

static void csip_lock_set_discover_sets_cb(struct bt_conn *conn, int err,
					   uint8_t set_count,
					   struct bt_csip_set_t *sets)
{
	int cb_err;
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		busy = false;
		cur_inst = NULL;
		cb_err = err;
		if (csip_cbs && csip_cbs->lock_set) {
			csip_cbs->lock_set(cb_err);
		}
		return;
	}

	for (int i = 0; i < cur_set.set_size; i++) {
		if (!servers[i].set_member.conn) {
			bt_addr_le_to_str(&set_member_addrs[i].addr, addr,
					  sizeof(addr));
			BT_DBG("[%d]: Connecting to %s", i, log_strdup(addr));

			cb_err = bt_conn_le_create(&set_member_addrs[i].addr,
						   BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
						   &servers[i].set_member.conn);

			if (cb_err) {
				busy = false;
				cur_inst = NULL;
				if (csip_cbs && csip_cbs->lock_set) {
					csip_cbs->lock_set(cb_err);
				}
			}
			return;
		}
	}

	/* Else everything is connected, start locking sets by rank */

	/* Make sure that the set is not locked */
	if (csip_set_is_locked()) {
		busy = false;
		cur_inst = NULL;
		cb_err = -EBUSY;
		if (csip_cbs && csip_cbs->lock_set) {
			csip_cbs->lock_set(cb_err);
		}
		return;
	}

	csip_write_lowest_rank();
}

static void csip_lock_set_init_cb(struct bt_conn *conn, int err,
				  uint8_t set_count)
{
	BT_DBG("");
	if (err || !conn) {
		BT_DBG("Could not connect");
	} else {
		discover_sets_cb = csip_lock_set_discover_sets_cb;
		/* Start reading values and call CB when done */
		err = read_set_sirk(conn, 0);
		if (err) {
			if (csip_cbs && csip_cbs->lock_set) {
				csip_cbs->lock_set(err);
			}
		}
	}
}

static void discover_members_timer_handler(struct k_work *work)
{
	BT_DBG("Could not find all members");
	bt_le_scan_stop();
	busy = false;
	cur_inst = NULL;
	if (csip_cbs && csip_cbs->members) {
		csip_cbs->members(-ETIMEDOUT, cur_set.set_size, members_found);
	}
}


static uint8_t csip_read_lock_cb(struct bt_conn *conn, uint8_t err,
				 struct bt_gatt_read_params *params,
				 const void *data, uint16_t length)
{
	uint8_t value = 0;
	int cb_err = err;
	uint8_t idx = cur_inst->idx;

	busy = false;

	if (err) {
		BT_DBG("err: 0x%02X", err);
	} else if (data) {
		if (length == sizeof(cur_inst->set_lock)) {
			bool locked;

			memcpy(&value, data, length);
			if (value != BT_CSIP_RELEASE_VALUE &&
			    value != BT_CSIP_LOCK_VALUE) {
				BT_DBG("Invalid value %u", value);
				return BT_GATT_ITER_STOP;
			}

			memcpy(&cur_inst->set_lock, data, length);

			locked = cur_inst->set_lock == BT_CSIP_LOCK_VALUE;
			BT_DBG("Instance %u lock is %s",
			       cur_inst->idx, locked ? "locked" : "released");
		} else {
			BT_DBG("Invalid length %u", length);
			cb_err = BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
		}
	}

	cur_inst = NULL;

	if (csip_cbs->lock_read) {
		csip_cbs->lock_read(conn, cb_err, idx,
				    value == BT_CSIP_LOCK_VALUE ? true : false);
	}
	return BT_GATT_ITER_STOP;
}

static void csip_write_lock_cb(struct bt_conn *conn, uint8_t err,
			       struct bt_gatt_write_params *params)
{
	uint8_t idx = cur_inst->idx;

	busy = false;
	cur_inst = NULL;

	if (err) {
		BT_DBG("Could not lock set (%d)", err);
	}

	if (csip_cbs && csip_cbs->lock) {
		csip_cbs->lock(conn, err, idx);
	}
}

static void csip_write_release_cb(struct bt_conn *conn, uint8_t err,
				  struct bt_gatt_write_params *params)
{
	uint8_t idx = cur_inst->idx;

	busy = false;
	cur_inst = NULL;

	if (err) {
		BT_DBG("Could not release set (%d)", err);
	}

	if (csip_cbs && csip_cbs->release) {
		csip_cbs->release(conn, err, idx);
	}
}

/*************************** PUBLIC FUNCTIONS ***************************/
void bt_csip_register_cb(struct bt_csip_cb_t *cb)
{
	csip_cbs = cb;
}

int bt_csip_discover(struct bt_conn *conn, bool subscribe)
{
	int err;
	static bool conn_cb_registered;
	struct bt_csis_server *server;

	if (!conn) {
		return -ENOTCONN;
	} else if (busy) {
		return -EBUSY;
	}

	if (!conn_cb_registered) {
		bt_conn_cb_register(&csip_conn_callbacks);
		bt_le_scan_cb_register(&csip_scan_callbacks);
		conn_cb_registered = true;
	}

	server = &servers[bt_conn_index(conn)];

	memset(server, 0, sizeof(*server));

	k_work_init_delayable(&discover_members_timer,
			      discover_members_timer_handler);

	server->set_member.conn = conn;
	memcpy(&set_member_addrs[0].addr,
	       bt_conn_get_dst(conn),
	       sizeof(bt_addr_le_t));
	if (csip_cbs && csip_cbs->discover) {
		err = init_discovery(server, subscribe, csip_cbs->discover);
	} else {
		err = init_discovery(server, subscribe, NULL);
	}
	if (!err) {
		busy = true;
	}
	return err;
}

int bt_csip_discover_sets(struct bt_conn *conn)
{
	int err;

	if (!conn) {
		BT_DBG("Not connected");
		return -ENOTCONN;
	} else if (busy) {
		return -EBUSY;
	}

	if (csip_cbs && csip_cbs->sets) {
		discover_sets_cb = csip_cbs->sets;
	}
	/* Start reading values and call CB when done */
	err = read_set_sirk(conn, 0);
	if (!err) {
		busy = true;
	}
	return err;
}

int bt_csip_discover_members(struct bt_csip_set_t *set)
{
	int err;

	if (busy) {
		return -EBUSY;
	} else if (!set) {
		return -EINVAL;
	} else if (set->set_size && set->set_size > ARRAY_SIZE(servers)) {
		/*
		 * TODO Handle case where set size is larger than
		 * number of possible connections
		 */
		BT_DBG("Set size (%u) larger than max connections (%u)",
		       set->set_size, (uint32_t)ARRAY_SIZE(servers));
		return -EINVAL;
	}

	memcpy(&cur_set, set, sizeof(cur_set));
	members_found = 0;

	err = k_work_reschedule(&discover_members_timer,
				CSIP_DISCOVER_TIMER_VALUE);
	if (err < 0) { /* Can return 0, 1 and 2 for success */
		BT_DBG("Could not schedule discover_members_timer %d", err);
		return err;
	}


	/*
	 * First member may be the currently connected device, and
	 * we are unlikely to see advertisements from that device
	 */
	for (int i = 0; i < ARRAY_SIZE(servers); i++) {
		if (servers[i].set_member.conn) {
			members_found++;
		}
	}

	if (members_found == cur_set.set_size) {
		if (csip_cbs && csip_cbs->members) {
			csip_cbs->members(0, cur_set.set_size, members_found);
		}

		return 0;
	}

	/* TODO: Add timeout on scan if not all devices could be found */
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (!err) {
		busy = true;
	}
	return err;
}

int bt_csip_lock_set(void)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bool fully_connected = true;

	/* TODO: Check ranks of devices in sets for duplicates */

	if (busy) {
		return -EBUSY;
	}

	busy = true;

	for (int i = 0; i < cur_set.set_size; i++) {
		if (!servers[i].set_member.conn) {
			int err;

			bt_addr_le_to_str(&set_member_addrs[i].addr, addr,
					  sizeof(addr));
			BT_DBG("[%d]: Connecting to %s", i, log_strdup(addr));


			err = bt_conn_le_create(&set_member_addrs[i].addr,
						BT_CONN_LE_CREATE_CONN,
						BT_LE_CONN_PARAM_DEFAULT,
						&servers[i].set_member.conn);
			if (err) {
				return err;
			}

			fully_connected = false;
			break;
		}
	}

	if (fully_connected) {
		/* Make sure that the set is not locked */
		if (csip_set_is_locked()) {
			busy = false;
			return -EAGAIN;
		}

		csip_write_lowest_rank();
	}

	return 0;
}

int bt_csip_release_set(void)
{
	csip_release_highest_rank();
	return 0;
}

int bt_csip_disconnect(void)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	for (int i = 0; i < cur_set.set_size; i++) {
		BT_DBG("member %d", i);
		if (servers[i].set_member.conn) {
			bt_addr_le_to_str(&set_member_addrs[i].addr, addr,
					  sizeof(addr));
			BT_DBG("Disconnecting %s", log_strdup(addr));
			err = bt_conn_disconnect(
				servers[i].set_member.conn,
				BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			servers[i].set_member.conn = NULL;
			if (err) {
				return err;
			}
		}
	}
	return 0;
}

int bt_csip_lock_get(struct bt_conn *conn, uint8_t inst_idx)
{
	int err;

	if (inst_idx >= CONFIG_BT_CSIP_MAX_CSIS_INSTANCES) {
		BT_DBG("Invalid index %u", inst_idx);
		return -EINVAL;
	} else if (busy) {
		BT_DBG("CSIP busy");
		return -EBUSY;
	} else if (cur_inst) {
		if (cur_inst != lookup_instance_by_index(conn, inst_idx)) {
			BT_DBG("CSIP busy with current instance");
			return -EBUSY;
		}
	} else {
		cur_inst = lookup_instance_by_index(conn, inst_idx);
		if (!cur_inst) {
			BT_DBG("Inst not found");
			return -EINVAL;
		}
	}

	if (!cur_inst->set_lock_handle) {
		BT_DBG("Handle not set");
		cur_inst = NULL;
		return -EINVAL;
	}

	read_params.func = csip_read_lock_cb;
	read_params.handle_count = 1;
	read_params.single.handle = cur_inst->set_lock_handle;
	read_params.single.offset = 0U;

	err = bt_gatt_read(conn, &read_params);

	if (err) {
		cur_inst = NULL;
	} else {
		busy = true;
	}

	return err;
}

int bt_csip_lock(struct bt_conn *conn, uint8_t inst_idx)
{
	int err;

	if (cur_inst || busy) {
		BT_DBG("CSIP busy");
		return -EBUSY;
	}

	err = csip_write_set_lock(conn, inst_idx, true, csip_write_lock_cb);
	if (err) {
		cur_inst = NULL;
	} else {
		busy = true;
	}

	return err;
}

int bt_csip_release(struct bt_conn *conn, uint8_t inst_idx)
{
	int err;

	if (cur_inst || busy) {
		BT_DBG("CSIP busy");
		return -EBUSY;
	}

	err = csip_write_set_lock(conn, inst_idx, false, csip_write_release_cb);
	if (err) {
		cur_inst = NULL;
	} else {
		busy = true;
	}

	return err;
}
