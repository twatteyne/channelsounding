/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Reflector with Ranging Responder sample
 *
 *  Source layout:
 *   - main.c       : top-level sequence
 *   - connection.c : advertising and connection handling
 *   - cs_setup.c   : CS event callbacks, default settings, procedure parameters
 *
 *  The Ranging Responder (RRSP) itself is handled by the RAS library.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#include "connection.h"
#include "cs_setup.h"

int main(void)
{
	int err;

	LOG_INF("Starting Channel Sounding Reflector Sample");

	dk_leds_init();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	if (connection_advertising_start()) {
		return 0;
	}

	while (true) {
		struct bt_conn *conn = connection_wait_connected();

		(void)cs_setup_default_settings(conn);

		cs_setup_wait_config();

		if (cs_setup_procedure_parameters(conn)) {
			return 0;
		}
	}

	return 0;
}
