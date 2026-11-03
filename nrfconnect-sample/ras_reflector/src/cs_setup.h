/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding setup on the reflector side.
 *
 *  The initiator drives the CS configuration; the reflector only enables its
 *  role and sets its procedure parameters once the configuration exists.
 */

#ifndef CS_SETUP_H_
#define CS_SETUP_H_

#include <zephyr/bluetooth/conn.h>

/** Enable the reflector role in the local controller. */
int cs_setup_default_settings(struct bt_conn *conn);

/** Block until the initiator has created the CS configuration. */
void cs_setup_wait_config(void);

/** Set the reflector procedure parameters. */
int cs_setup_procedure_parameters(struct bt_conn *conn);

#endif /* CS_SETUP_H_ */
