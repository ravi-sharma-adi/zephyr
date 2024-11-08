/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018-2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/net/lwm2m.h>
#include <zephyr/sys_clock.h>

#define LWM2M_PACKAGE_URI_LEN CONFIG_LWM2M_SWMGMT_PACKAGE_URI_LEN

struct requesting_object {
	uint8_t obj_inst_id;
	bool is_firmware_uri;

	void (*result_cb)(uint16_t obj_inst_id, int error_code);
	lwm2m_engine_set_data_cb_t write_cb;
	int (*verify_cb)(void);
};

/*
 * The pull context is also used in the LWM2M's Software Management object.
 * This means that the transfer needs to know if it's used for firmware or
 * something else.
 */
int lwm2m_pull_context_start_transfer(char *uri, struct requesting_object req, k_timeout_t timeout);

/**
 * @brief Set the @ref lwm2m_ctx::set_socketoptions callback for the pull context's client.
 *
 * This callback will be called after the pull context socket is created and before it
 * is connected.
 *
 * @param[in] set_sockopt_cb A callback function to set sockopts for the pull context client.
 */
void lwm2m_pull_context_set_sockopt_callback(lwm2m_set_sockopt_cb_t set_sockopt_cb);
