/*
 * Copyright (c) 2022 Abel Sensors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <net/socket.h>
#include <drivers/modem/ublox-sara-n310.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#define N310_DEVICE_LABEL "SARAN310"
#define MAX_BUF 512

/* PSM TIME SETTINGS */
#define PERIODIC_TAU "01011111"
#define ACTIVE_TIME "00000000"

void main(void)
{
	static const struct device *dev_ubloxn3;
	struct sockaddr_in addr;
	int n = 0, sockfd = 0;

	dev_ubloxn3 = device_get_binding(N310_DEVICE_LABEL);

	if (dev_ubloxn3 == NULL) {
		LOG_ERR("Failed to get device binding.");
		return;
	}

	LOG_INF("Manufacturer: %s", log_strdup(n310_get_manufacturer()));
	LOG_INF("Model: %s", log_strdup(n310_get_model()));
	LOG_INF("Revision: %s", log_strdup(n310_get_revision()));
	LOG_INF("ICCID: %s", log_strdup(n310_get_iccid()));
	LOG_INF("IMEI: %s", log_strdup(n310_get_imei()));
	LOG_INF("IP: %s", log_strdup(n310_get_ip()));

	int network_state = n310_get_network_state();
	LOG_INF("Network state: %d", network_state);

	int jamming_state = n310_get_jamming_state();
	LOG_INF("Jamming state: %d", jamming_state);

	int antenna_load = n310_get_antenna_load();
	LOG_INF("Antenna load: %d", antenna_load);

	struct signal_quality buffer;
	n310_get_signal_quality(&buffer);
	LOG_INF("Signal quality: %d, %d", buffer.rssi, buffer.quality);

	n310_psm_set_csclk(2);
	n310_psm_config(PSM_ENABLE, PERIODIC_TAU, ACTIVE_TIME);
	n310_psm_set_mode(PM2);

	n310_set_last_gasp("Hello there!", "192.168.0.10:5050", 1, 1);
	n310_send_last_gasp();

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(4242);
	inet_pton(AF_INET, "192.168.0.20", &addr.sin_addr);

	sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sockfd < 0) {
		LOG_INF("Could not create socket. Return value: %d", sockfd);
	}

	LOG_INF("Connecting...");
	n = connect(sockfd, (const struct sockaddr *)&addr, sizeof(addr));
	if (n < 0) {
	  LOG_ERR("Bind failed with return value: %d", n);
	}

	while (1) {
		char *buffer = "ABDD";
		char recvBuffer[MAX_BUF] = "\0";

		LOG_INF("Sending...");
		n = sendto(sockfd, buffer, strlen(buffer) / 2,
			   RELEASE_AFTER_FIRST_DOWNLINK,
			   (const struct sockaddr *)&addr, sizeof(addr));
		if (n < 0) {
		  LOG_ERR("Send failed with return value: %d", n);
		}

		k_msleep(5000);

		LOG_INF("Receiving...");
		n = recvfrom(sockfd, recvBuffer, MAX_BUF, ZSOCK_MSG_DONTWAIT,
			     NULL, NULL);
		if (n <= 0) {
			strcpy(recvBuffer, "No data in buffer.");
		}
		LOG_INF("Received: %s", log_strdup(recvBuffer));

		k_msleep(2000);
	}
}
