/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Phase-Based Ranging initiator.
 *
 *  1. Connects to a reflector that advertises the Ranging Service (RAS).
 *  2. Creates a Channel Sounding configuration in main mode 2 (PBR tones).
 *  3. At each CS procedure, collects its own PCTs (local step data) and those
 *     of the reflector (RAS ranging data) into a cs_de report
 *     (../lib/cs_ranging.c), then computes the distance with the Nordic
 *     cs_de library (phase slope + IFFT) in the main thread.
 *
 * The Bluetooth part (connection, RAS, CS configuration) is derived from the
 * NCS sample channel_sounding/ras_initiator.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <dk_buttons_and_leds.h>

#include "cs_ranging.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pbr_initiator, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1

#define CS_CONFIG_ID           0
#define NUM_MODE_0_STEPS       3
#define PROCEDURE_COUNTER_NONE (-1)

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_ras_features, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);

/* Procedures ready to be processed by the main thread (copy of the report). */
K_MSGQ_DEFINE(proc_msgq, sizeof(struct cs_ranging_proc), 2, 4);

static struct bt_conn *connection;
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static uint32_t ras_feature_bits;
static struct bt_conn_le_cs_config cs_config;

/* Procedure being parsed (Bluetooth thread). */
static struct cs_ranging_proc rx_proc;

/* ------------------------------------------------------------------------- */
/* Step data extraction                                                        */

static bool process_ranging_header(struct ras_ranging_header *ranging_header, void *user_data)
{
	struct cs_ranging_proc *proc = user_data;
	uint8_t n_ap = (uint8_t)__builtin_popcount(ranging_header->antenna_paths_mask & 0x0F);

	if (n_ap > CS_RANGING_MAX_AP) {
		LOG_WRN("%u antenna paths in the data, only %u handled", n_ap, CS_RANGING_MAX_AP);
	}

	cs_ranging_reset(proc, cs_config.role, n_ap);

	return true;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	struct cs_ranging_proc *proc = user_data;

	if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		cs_ranging_add_mode2_step(
			proc, local_step->channel,
			(const struct bt_hci_le_cs_step_data_mode_2 *)local_step->data,
			(const struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data);
	}
	/* Mode 0 (calibration) steps are ignored. */

	return true;
}

/* ------------------------------------------------------------------------- */
/* RAS / CS callbacks                                                           */

static void release_local_steps(void)
{
	net_buf_simple_reset(&latest_local_steps);
	if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
		net_buf_simple_reset(&latest_peer_steps);
	}
	k_sem_give(&sem_local_steps);
}

static void ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	ARG_UNUSED(conn);

	if (err) {
		LOG_ERR("Error receiving ranging data, counter %u (err %d)", ranging_counter, err);
		return;
	}

	if (ranging_counter != most_recent_local_ranging_counter) {
		LOG_INF("Ranging data dropped: counter mismatch (peer %u, local %d)",
			ranging_counter, most_recent_local_ranging_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
		return;
	}

	if (latest_local_steps.len == 0) {
		LOG_WRN("All subevents of procedure %u were aborted", ranging_counter);
		release_local_steps();
		return;
	}

	/* Reset here too in case the ranging header is missing. */
	cs_ranging_reset(&rx_proc, cs_config.role, 1);
	bt_ras_rreq_rd_subevent_data_parse(&latest_peer_steps, &latest_local_steps,
					   cs_config.role, process_ranging_header, NULL,
					   process_step_data, &rx_proc);
	release_local_steps();

	/* cs_de runs in the main thread, not in the Bluetooth RX thread. */
	if (k_msgq_put(&proc_msgq, &rx_proc, K_NO_WAIT) != 0) {
		LOG_DBG("Computation busy, procedure %u ignored", ranging_counter);
	}
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	ARG_UNUSED(conn);

	if (dropped_ranging_counter == result->header.procedure_counter) {
		return;
	}

	if (most_recent_local_ranging_counter !=
	    bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
		if (k_sem_take(&sem_local_steps, K_NO_WAIT) < 0) {
			dropped_ranging_counter = result->header.procedure_counter;
			LOG_DBG("Subevent results dropped, waiting for peer ranging data.");
			return;
		}

		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* Steps of an aborted subevent are not used. */
	} else if (result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory to store step data (%u > %u)",
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

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	ARG_UNUSED(conn);

	if (ranging_counter != most_recent_local_ranging_counter) {
		return;
	}

	int err = bt_ras_rreq_cp_get_ranging_data(connection, &latest_peer_steps, ranging_counter,
						  ranging_data_cb);
	if (err) {
		LOG_ERR("Get ranging data failed (err %d)", err);
		net_buf_simple_reset(&latest_local_steps);
		net_buf_simple_reset(&latest_peer_steps);
		k_sem_give(&sem_local_steps);
	}
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	ARG_UNUSED(conn);
	LOG_INF("Ranging data overwritten %u", ranging_counter);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
		return;
	}

	LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
	k_sem_give(&sem_mtu_exchange_done);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	ARG_UNUSED(context);

	int err = bt_ras_rreq_alloc_and_assign_handles(dm, bt_gatt_dm_conn_get(dm));

	if (err) {
		LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data (err %d)", err);
	}

	LOG_INF("RAS discovery done");
	k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_INF("Ranging Service not found, disconnecting");
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_INF("Discovery failed (err %d)", err);
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_ERR("Security failed: level %u err %d", level, err);
		return;
	}

	LOG_INF("Security changed: level %u", level);
	k_sem_give(&sem_security);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	/* Ignore the peer's parameter preferences. */
	return false;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s (err 0x%02X)", addr, err);

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
		return;
	}

	connection = bt_conn_ref(conn);
	k_sem_give(&sem_connected);
	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02X)", reason);

	bt_conn_unref(conn);
	connection = NULL;
	dk_set_led_off(CON_STATUS_LED);

	sys_reboot(SYS_REBOOT_COLD);
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS capability exchange completed.");
		k_sem_give(&sem_remote_capabilities_obtained);
	} else {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
	}
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		cs_config = *config;
		LOG_INF("CS config created: id %u, main mode %u, steps %u-%u, "
			"T_IP2 %u us, T_PM %u us",
			config->id, config->mode, config->min_main_mode_steps,
			config->max_main_mode_steps, config->t_ip2_time_us, config->t_pm_time_us);
		k_sem_give(&sem_config_created);
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS security enabled.");
		k_sem_give(&sem_cs_security_enabled);
	} else {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);

	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
	} else if (params->state == 1) {
		LOG_INF("CS procedures enabled (interval %u, subevent %u us, TX %d dBm).",
			params->procedure_interval, params->subevent_len,
			params->selected_tx_power);
	} else {
		LOG_INF("CS procedures disabled.");
	}
}

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

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req,
	.security_changed = security_changed,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

/* ------------------------------------------------------------------------- */
/* Scanning                                                                    */

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("Reflector found: %s (connectable %d)", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_INF("Connecting failed, restarting scanning");

	int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);

	if (err) {
		LOG_ERR("Failed to restart scanning (err %d)", err);
	}
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, NULL);

static int scan_init(struct bt_scan_init_param *p_param)
{
	int err;

	bt_scan_init(p_param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
	}

	return err;
}

/* ------------------------------------------------------------------------- */
/* Distance post-processing                                                    */

static void distance_loop(void)
{
	static struct cs_ranging_proc proc;
	static struct cs_ranging_median median[CS_RANGING_MAX_AP];
	struct cs_ranging_estimate est[CS_RANGING_MAX_AP];
	const float offset_m = (float)CONFIG_PBR_CALIBRATION_OFFSET_CM / 100.0f;

	for (int ap = 0; ap < CS_RANGING_MAX_AP; ap++) {
		cs_ranging_median_init(&median[ap], CONFIG_PBR_MEDIAN_WINDOW);
	}

	while (true) {
		k_msgq_get(&proc_msgq, &proc, K_FOREVER);

		uint32_t t0 = k_cycle_get_32();
		int n_valid = cs_ranging_compute(&proc, CONFIG_PBR_MIN_CHANNELS, offset_m, est);
		uint32_t dt_us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

		if (n_valid == 0) {
			LOG_WRN("No estimate: %u channels usable (min %u), %u tones rejected",
				est[0].n_channels, CONFIG_PBR_MIN_CHANNELS, proc.tones_rejected);
			continue;
		}

		for (uint8_t ap = 0; ap < proc.report.n_ap; ap++) {
			if (!est[ap].valid) {
				continue;
			}

			float filtered = cs_ranging_median_push(&median[ap], est[ap].best_m);

			LOG_INF("AP%u distance %.2f m (median of %u) | ifft %.2f m, "
				"phase slope %.2f m | %u channels, %u us",
				ap, (double)filtered, median[ap].len, (double)est[ap].ifft_m,
				(double)est[ap].phase_slope_m, est[ap].n_channels, dt_us);
		}
	}
}

/* ------------------------------------------------------------------------- */

static void cs_config_get(struct bt_le_cs_create_config_params *p)
{
	memset(p, 0, sizeof(*p));
	p->id = CS_CONFIG_ID;
	/* Main mode 2 = tone exchange only (PBR). */
	p->mode = BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE;
	p->min_main_mode_steps = 2;
	p->max_main_mode_steps = 5;
	p->main_mode_repetition = 0;
	p->mode_0_steps = NUM_MODE_0_STEPS;
	p->role = BT_CONN_LE_CS_ROLE_INITIATOR;
	p->rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	p->cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY;
	/* Each channel of the map is visited once per procedure. */
	p->channel_map_repetition = 1;
	p->channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	p->ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT;
	p->ch3c_jump = 2;
	/* All allowed channels (2..76 minus 22..26 for advertising). */
	bt_le_cs_set_valid_chmap_bits(p->channel_map);
}

int main(void)
{
	int err;

	LOG_INF("Starting Phase-Based Ranging initiator");

	dk_leds_init();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	struct bt_scan_init_param scan_params = {
		.scan_param = NULL,
		.conn_param = BT_LE_CONN_PARAM(0x10, 0x10, 0, BT_GAP_MS_TO_CONN_TIMEOUT(4000)),
		.connect_if_match = 1,
	};

	err = scan_init(&scan_params);
	if (err) {
		return 0;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_connected, K_FOREVER);

	err = bt_conn_set_security(connection, BT_SECURITY_L2);
	if (err) {
		LOG_ERR("Failed to encrypt connection (err %d)", err);
		return 0;
	}
	k_sem_take(&sem_security, K_FOREVER);

	static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

	bt_gatt_exchange_mtu(connection, &mtu_exchange_params);
	k_sem_take(&sem_mtu_exchange_done, K_FOREVER);

	err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("Discovery failed (err %d)", err);
		return 0;
	}
	k_sem_take(&sem_discovery_done, K_FOREVER);

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(connection, &default_settings);
	if (err) {
		LOG_ERR("Failed to configure default CS settings (err %d)", err);
		return 0;
	}

	err = bt_ras_rreq_read_features(connection, ras_features_read_cb);
	if (err) {
		LOG_ERR("Could not get RAS features from peer (err %d)", err);
		return 0;
	}
	k_sem_take(&sem_ras_features, K_FOREVER);

	const bool realtime_rd = ras_feature_bits & RAS_FEAT_REALTIME_RD;

	if (realtime_rd) {
		err = bt_ras_rreq_realtime_rd_subscribe(connection, &latest_peer_steps,
							ranging_data_cb);
		if (err) {
			LOG_ERR("Real-time ranging data subscribe failed (err %d)", err);
			return 0;
		}
	} else {
		err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
		err = err ?: bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
		err = err ?: bt_ras_rreq_on_demand_rd_subscribe(connection);
		err = err ?: bt_ras_rreq_cp_subscribe(connection);
		if (err) {
			LOG_ERR("On-demand ranging data subscribe failed (err %d)", err);
			return 0;
		}
	}

	err = bt_le_cs_read_remote_supported_capabilities(connection);
	if (err) {
		LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
		return 0;
	}
	k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

	struct bt_le_cs_create_config_params config_params;

	cs_config_get(&config_params);

	err = bt_le_cs_create_config(connection, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Failed to create CS config (err %d)", err);
		return 0;
	}
	k_sem_take(&sem_config_created, K_FOREVER);

	err = bt_le_cs_security_enable(connection);
	if (err) {
		LOG_ERR("Failed to start CS Security (err %d)", err);
		return 0;
	}
	k_sem_take(&sem_cs_security_enabled, K_FOREVER);

	/* Conversion from connection interval units (1.25 ms) to procedure units (0.625 ms). */
	const uint16_t acl_interval_in_proc_interval_units =
		scan_params.conn_param->interval_max * 2;
	const uint16_t procedure_interval = realtime_rd ? 5 : 10;

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = acl_interval_in_proc_interval_units * (procedure_interval - 1),
		.min_procedure_interval = procedure_interval,
		.max_procedure_interval = procedure_interval,
		.max_procedure_count = 0,
		.min_subevent_len = 16000,
		.max_subevent_len = 16000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_2M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
	if (err) {
		LOG_ERR("Failed to set procedure parameters (err %d)", err);
		return 0;
	}

	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = 1,
	};

	err = bt_le_cs_procedure_enable(connection, &params);
	if (err) {
		LOG_ERR("Failed to enable CS procedures (err %d)", err);
		return 0;
	}

	distance_loop();

	return 0;
}
