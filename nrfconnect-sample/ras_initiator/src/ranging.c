/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/ras.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#include "cs_processing.h"
#include "cs_setup.h"
#include "csv_export.h"
#include "distance_estimate.h"
#include "ranging.h"

#define PROCEDURE_COUNTER_NONE (-1)

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_ras_features, 0, 1);
/* Taken while the local step buffer holds a procedure waiting for peer data. */
static K_SEM_DEFINE(sem_local_steps, 1, 1);

NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static uint32_t ras_feature_bits;

static void release_step_buffers(void)
{
	net_buf_simple_reset(&latest_local_steps);

	if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
		net_buf_simple_reset(&latest_peer_steps);
	}

	k_sem_give(&sem_local_steps);
}

/* ---------------------------------------------------------------------------
 * Peer ranging data
 * ------------------------------------------------------------------------- */

static void ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	ARG_UNUSED(conn);

	if (err) {
		LOG_ERR("Error when receiving ranging data with ranging counter %d (err %d)",
			ranging_counter, err);
		return;
	}

	if (ranging_counter != most_recent_local_ranging_counter) {
		LOG_INF("Ranging data dropped as peer ranging counter doesn't match local ranging "
			"data counter. (peer: %u, local: %u)",
			ranging_counter, most_recent_local_ranging_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
		return;
	}

	LOG_DBG("Ranging data received for ranging counter %d", ranging_counter);

	if (latest_local_steps.len == 0) {
		LOG_WRN("All subevents in ranging counter %u were aborted",
			most_recent_local_ranging_counter);
		release_step_buffers();
		return;
	}

	cs_processing_extract(&latest_peer_steps, &latest_local_steps, cs_setup_get_config());

	/* Step data has been consumed: the buffers can receive the next procedure. */
	release_step_buffers();

	cs_de_quality_t quality = cs_processing_estimate();
	const struct cs_procedure_result *result = cs_processing_result();

	csv_export_submit(ranging_counter, result);

	if (quality == CS_DE_QUALITY_OK) {
		distance_estimate_update(&result->report);
	}
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_DBG("Ranging data ready %i", ranging_counter);

	if (ranging_counter == most_recent_local_ranging_counter) {
		int err = bt_ras_rreq_cp_get_ranging_data(conn, &latest_peer_steps,
							  ranging_counter,
							  ranging_data_cb);
		if (err) {
			LOG_ERR("Get ranging data failed (err %d)", err);
			net_buf_simple_reset(&latest_local_steps);
			net_buf_simple_reset(&latest_peer_steps);
			k_sem_give(&sem_local_steps);
		}
	}
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_INF("Ranging data overwritten %i", ranging_counter);
}

/* ---------------------------------------------------------------------------
 * Local CS subevent results
 * ------------------------------------------------------------------------- */

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	if (dropped_ranging_counter == result->header.procedure_counter) {
		return;
	}

	if (most_recent_local_ranging_counter
		!= bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
		int sem_state = k_sem_take(&sem_local_steps, K_NO_WAIT);

		if (sem_state < 0) {
			dropped_ranging_counter = result->header.procedure_counter;
			LOG_INF("Dropped subevent results. Waiting for ranging data from peer.");
			return;
		}

		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* The steps from this subevent will not be used. */
	} else if (result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory to store step data. (%d > %d)",
				latest_local_steps.len + result->step_data_buf->len,
				latest_local_steps.size);
			net_buf_simple_reset(&latest_local_steps);
			dropped_ranging_counter = result->header.procedure_counter;
			return;
		}
	}

	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		LOG_WRN("Procedure %u aborted", result->header.procedure_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
	}
}

BT_CONN_CB_DEFINE(ranging_conn_cb) = {
	.le_cs_subevent_data_available = subevent_result_cb,
};

/* ---------------------------------------------------------------------------
 * Subscription
 * ------------------------------------------------------------------------- */

static void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
	if (err) {
		LOG_WRN("Error while reading RAS feature bits (err %d)", err);
	} else {
		LOG_INF("Read RAS feature bits: 0x%x", feature_bits);
		ras_feature_bits = feature_bits;
	}

	k_sem_give(&sem_ras_features);
}

int ranging_subscribe(struct bt_conn *conn, bool *realtime_rd)
{
	int err;

	err = bt_ras_rreq_read_features(conn, ras_features_read_cb);
	if (err) {
		LOG_ERR("Could not get RAS features from peer (err %d)", err);
		return err;
	}

	k_sem_take(&sem_ras_features, K_FOREVER);

	*realtime_rd = ras_feature_bits & RAS_FEAT_REALTIME_RD;

	if (*realtime_rd) {
		err = bt_ras_rreq_realtime_rd_subscribe(conn, &latest_peer_steps, ranging_data_cb);
		if (err) {
			LOG_ERR("RAS RREQ Real-time ranging data subscribe failed (err %d)", err);
			return err;
		}

		return 0;
	}

	err = bt_ras_rreq_rd_overwritten_subscribe(conn, ranging_data_overwritten_cb);
	if (err) {
		LOG_ERR("RAS RREQ ranging data overwritten subscribe failed (err %d)", err);
		return err;
	}

	err = bt_ras_rreq_rd_ready_subscribe(conn, ranging_data_ready_cb);
	if (err) {
		LOG_ERR("RAS RREQ ranging data ready subscribe failed (err %d)", err);
		return err;
	}

	err = bt_ras_rreq_on_demand_rd_subscribe(conn);
	if (err) {
		LOG_ERR("RAS RREQ On-demand ranging data subscribe failed (err %d)", err);
		return err;
	}

	err = bt_ras_rreq_cp_subscribe(conn);
	if (err) {
		LOG_ERR("RAS RREQ CP subscribe failed (err %d)", err);
		return err;
	}

	return 0;
}
