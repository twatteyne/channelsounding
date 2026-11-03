/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Ranging Requestor sample
 *
 *  Source layout:
 *   - main.c              : top-level sequence (connect, set up CS, print distances)
 *   - connection.c        : scanning, connection, security, MTU, GATT discovery
 *   - cs_setup.c          : CS default settings, config, security, procedure enable
 *   - ranging.c           : RAS RREQ, collection of local and peer step data
 *   - cs_processing.c     : tone / RTT / RSSI extraction and cs_de estimation
 *   - distance_estimate.c : sliding-window median of the distance estimates
 *   - csv_export.c        : CSV output of each procedure (see scripts/)
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#include "app_config.h"
#include "connection.h"
#include "cs_setup.h"
#include "distance_estimate.h"
#include "ranging.h"

int main(void)
{
	int err;
	bool realtime_rd;
	struct bt_conn *conn;

	LOG_INF("Starting Channel Sounding Initiator Sample");

	dk_leds_init();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	/* Connect to a Ranging Service peer. */
	if (connection_scan_start()) {
		return 0;
	}

	conn = connection_wait_connected();

	if (connection_secure(conn)) {
		return 0;
	}

	connection_exchange_mtu(conn);

	if (connection_discover_ras(conn)) {
		return 0;
	}

	/* Set up ranging data transfer and Channel Sounding. */
	if (cs_setup_default_settings(conn) ||
	    ranging_subscribe(conn, &realtime_rd) ||
	    cs_setup_read_remote_capabilities(conn) ||
	    cs_setup_create_config(conn) ||
	    cs_setup_enable_security(conn) ||
	    cs_setup_start_procedures(conn, realtime_rd)) {
		return 0;
	}

	/* Print the filtered distance each time a new estimate is available. */
	while (true) {
		distance_estimate_wait();
		for (uint8_t ap = 0; ap < MAX_AP; ap++) {
			if (distance_estimate_available(ap)) {
				distance_estimate_print(ap);
			}
		}
	}

	return 0;
}
