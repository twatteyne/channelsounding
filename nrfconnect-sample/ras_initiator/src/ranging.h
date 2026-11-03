/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Ranging Requestor (RREQ): collects local CS subevent results and the
 *         peer ranging data, then triggers processing for each procedure.
 */

#ifndef RANGING_H_
#define RANGING_H_

#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>

/** Read the peer RAS features and subscribe to the ranging data.
 *
 *  Real-time ranging data is used if supported by the peer, on-demand
 *  ranging data otherwise.
 *
 *  @param[out] realtime_rd true if real-time ranging data is used.
 */
int ranging_subscribe(struct bt_conn *conn, bool *realtime_rd);

#endif /* RANGING_H_ */
