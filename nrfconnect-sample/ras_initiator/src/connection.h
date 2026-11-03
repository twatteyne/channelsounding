/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Scanning, connection, security and GATT discovery of the RAS peer.
 */

#ifndef CONNECTION_H_
#define CONNECTION_H_

#include <zephyr/bluetooth/conn.h>

/** Initialize the scanner (filter on the Ranging Service UUID) and start scanning. */
int connection_scan_start(void);

/** Block until a peer is connected and return the connection. */
struct bt_conn *connection_wait_connected(void);

/** Encrypt the link (security level 2) and wait until it is done. */
int connection_secure(struct bt_conn *conn);

/** Exchange the ATT MTU and wait until it is done. */
void connection_exchange_mtu(struct bt_conn *conn);

/** Discover the peer Ranging Service, assign RAS RREQ handles and wait until done. */
int connection_discover_ras(struct bt_conn *conn);

#endif /* CONNECTION_H_ */
