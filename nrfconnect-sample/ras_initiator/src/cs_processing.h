/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Extraction of tones / RTT / RSSI from CS step data and distance estimation.
 */

#ifndef CS_PROCESSING_H_
#define CS_PROCESSING_H_

#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/cs_de.h>

#include "app_config.h"

/** Everything computed for one CS procedure. */
struct cs_procedure_result {
	/** IQ tones, RTT accumulation, tone quality and distance estimates. */
	cs_de_report_t report;
	/** Number of high-quality tones averaged per antenna path and channel. */
	uint16_t n_iqs[MAX_AP][CS_DE_NUM_CHANNELS];
	/** RSSI accumulated over the valid local steps. */
	int32_t rssi_sum;
	uint16_t rssi_count;
};

/** Parse the local and peer step data of one procedure into the result.
 *
 *  The step buffers are no longer needed once this function returns.
 */
void cs_processing_extract(struct net_buf_simple *peer_steps,
			   struct net_buf_simple *local_steps,
			   const struct bt_conn_le_cs_config *config);

/** Run the distance estimation (cs_de) on the extracted data. */
cs_de_quality_t cs_processing_estimate(void);

/** Result of the last processed procedure. */
const struct cs_procedure_result *cs_processing_result(void);

#endif /* CS_PROCESSING_H_ */
