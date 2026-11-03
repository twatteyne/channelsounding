/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Advertising of the Ranging Service and connection handling.
 */

#ifndef CONNECTION_H_
#define CONNECTION_H_

#include <zephyr/bluetooth/conn.h>

/** Start connectable advertising with the Ranging Service UUID. */
int connection_advertising_start(void);

/** Block until a peer is connected and return the connection. */
struct bt_conn *connection_wait_connected(void);

#endif /* CONNECTION_H_ */
