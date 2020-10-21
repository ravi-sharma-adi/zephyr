/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <ztest.h>
#include "kconfig.h"

#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <sys/slist.h>
#include <sys/util.h>

#include "hal/ccm.h"

#include "util/mem.h"
#include "util/memq.h"

#include "pdu.h"
#include "ll.h"
#include "ll_settings.h"
#include "ll_feat.h"

#include "lll.h"
#include "lll_conn.h"
#include "ull_tx_queue.h"
#include "ull_conn_types.h"

#include "ull_llcp.h"

extern sys_slist_t ut_rx_q;

int lll_csrand_get(void *buf, size_t len)
{
	* (int *) buf = 0;
	return 0;
}
