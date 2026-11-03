/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding configuration and procedure control (initiator side).
 *
 *  Every function blocks until the corresponding HCI event has been received.
 */

#ifndef CS_SETUP_H_
#define CS_SETUP_H_

#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>

/** Enable the initiator role in the local controller. */
int cs_setup_default_settings(struct bt_conn *conn);

/** Read the CS capabilities of the peer. */
int cs_setup_read_remote_capabilities(struct bt_conn *conn);

/** Create the CS configuration on both sides. */
int cs_setup_create_config(struct bt_conn *conn);

/** Start the CS security procedure. */
int cs_setup_enable_security(struct bt_conn *conn);

/** Set the procedure parameters and enable CS procedures.
 *
 *  @param realtime_rd true if the peer sends ranging data in real time
 *                     (allows a shorter procedure interval).
 */
int cs_setup_start_procedures(struct bt_conn *conn, bool realtime_rd);

/** CS configuration negotiated with the peer (valid after cs_setup_create_config()). */
const struct bt_conn_le_cs_config *cs_setup_get_config(void);

#endif /* CS_SETUP_H_ */
