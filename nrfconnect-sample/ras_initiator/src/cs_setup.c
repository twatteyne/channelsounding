/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/conn.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#include "app_config.h"
#include "cs_setup.h"

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);

static struct bt_conn_le_cs_config cs_config;

/* ---------------------------------------------------------------------------
 * CS event callbacks
 * ------------------------------------------------------------------------- */

static void remote_capabilities_cb(struct bt_conn *conn,
				   uint8_t status,
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

static void cs_config_print(const struct bt_conn_le_cs_config *config)
{
	const char *mode_str[5] = {"Unused", "1 (RTT)", "2 (PBR)", "3 (RTT + PBR)", "Invalid"};
	const char *role_str[3] = {"Initiator", "Reflector", "Invalid"};
	const char *rtt_type_str[8] = {
		"AA only",	 "32-bit sounding", "96-bit sounding", "32-bit random",
		"64-bit random", "96-bit random",   "128-bit random",  "Invalid"};
	const char *phy_str[4] = {"Invalid", "LE 1M PHY", "LE 2M PHY", "LE 2M 2BT PHY"};
	const char *chsel_type_str[3] = {"Algorithm #3b", "Algorithm #3c", "Invalid"};
	const char *ch3c_shape_str[3] = {"Hat shape", "X shape", "Invalid"};

	uint8_t mode_idx = config->mode > 0 && config->mode < 4 ? config->mode : 4;
	uint8_t role_idx = MIN(config->role, 2);
	uint8_t rtt_type_idx = MIN(config->rtt_type, 7);
	uint8_t phy_idx = config->cs_sync_phy > 0 && config->cs_sync_phy < 4
				  ? config->cs_sync_phy
				  : 0;
	uint8_t chsel_type_idx = MIN(config->channel_selection_type, 2);
	uint8_t ch3c_shape_idx = MIN(config->ch3c_shape, 2);

	LOG_INF("CS config creation complete.\n"
		" - id: %u\n"
		" - mode: %s\n"
		" - min_main_mode_steps: %u\n"
		" - max_main_mode_steps: %u\n"
		" - main_mode_repetition: %u\n"
		" - mode_0_steps: %u\n"
		" - role: %s\n"
		" - rtt_type: %s\n"
		" - cs_sync_phy: %s\n"
		" - channel_map_repetition: %u\n"
		" - channel_selection_type: %s\n"
		" - ch3c_shape: %s\n"
		" - ch3c_jump: %u\n"
		" - t_ip1_time_us: %u\n"
		" - t_ip2_time_us: %u\n"
		" - t_fcs_time_us: %u\n"
		" - t_pm_time_us: %u\n"
		" - channel_map: 0x%08X%08X%04X\n",
		config->id, mode_str[mode_idx],
		config->min_main_mode_steps, config->max_main_mode_steps,
		config->main_mode_repetition, config->mode_0_steps, role_str[role_idx],
		rtt_type_str[rtt_type_idx], phy_str[phy_idx],
		config->channel_map_repetition, chsel_type_str[chsel_type_idx],
		ch3c_shape_str[ch3c_shape_idx], config->ch3c_jump, config->t_ip1_time_us,
		config->t_ip2_time_us, config->t_fcs_time_us, config->t_pm_time_us,
		sys_get_le32(&config->channel_map[6]),
		sys_get_le32(&config->channel_map[2]),
		sys_get_le16(&config->channel_map[0]));
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		cs_config = *config;
		cs_config_print(config);
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

static void procedure_enable_cb(struct bt_conn *conn,
				uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		if (params->state == 1) {
			LOG_INF("CS procedures enabled:\n"
				" - config ID: %u\n"
				" - antenna configuration index: %u\n"
				" - TX power: %d dbm\n"
				" - subevent length: %u us\n"
				" - subevents per event: %u\n"
				" - subevent interval: %u\n"
				" - event interval: %u\n"
				" - procedure interval: %u\n"
				" - procedure count: %u\n"
				" - maximum procedure length: %u",
				params->config_id, params->tone_antenna_config_selection,
				params->selected_tx_power, params->subevent_len,
				params->subevents_per_event, params->subevent_interval,
				params->event_interval, params->procedure_interval,
				params->procedure_count, params->max_procedure_len);
		} else {
			LOG_INF("CS procedures disabled.");
		}
	} else {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
	}
}

BT_CONN_CB_DEFINE(cs_setup_conn_cb) = {
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
};

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

const struct bt_conn_le_cs_config *cs_setup_get_config(void)
{
	return &cs_config;
}

int cs_setup_default_settings(struct bt_conn *conn)
{
	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	int err = bt_le_cs_set_default_settings(conn, &default_settings);

	if (err) {
		LOG_ERR("Failed to configure default CS settings (err %d)", err);
	}

	return err;
}

int cs_setup_read_remote_capabilities(struct bt_conn *conn)
{
	int err = bt_le_cs_read_remote_supported_capabilities(conn);

	if (err) {
		LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
		return err;
	}

	k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);
	return 0;
}

static void cs_config_get(struct bt_le_cs_create_config_params *config_params)
{
	memset(config_params, 0, sizeof(struct bt_le_cs_create_config_params));
	config_params->id = CS_CONFIG_ID;
	config_params->mode = CS_CONFIG_MODE;
	config_params->min_main_mode_steps = 2;
	config_params->max_main_mode_steps = 5;
	config_params->main_mode_repetition = 0;
	config_params->mode_0_steps = NUM_MODE_0_STEPS;
	config_params->role = BT_CONN_LE_CS_ROLE_INITIATOR;
	config_params->rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	config_params->cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY;
	config_params->channel_map_repetition = 1;
	config_params->channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	config_params->ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT;
	config_params->ch3c_jump = 2;
}

int cs_setup_create_config(struct bt_conn *conn)
{
	struct bt_le_cs_create_config_params config_params;

	cs_config_get(&config_params);

	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

	int err = bt_le_cs_create_config(conn, &config_params,
					 BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Failed to create CS config (err %d)", err);
		return err;
	}

	k_sem_take(&sem_config_created, K_FOREVER);
	return 0;
}

int cs_setup_enable_security(struct bt_conn *conn)
{
	int err = bt_le_cs_security_enable(conn);

	if (err) {
		LOG_ERR("Failed to start CS Security (err %d)", err);
		return err;
	}

	k_sem_take(&sem_cs_security_enabled, K_FOREVER);
	return 0;
}

int cs_setup_start_procedures(struct bt_conn *conn, bool realtime_rd)
{
	int err;

	/* scale factor of conn_interval units to proc_interval units is 1.25/0.625 = 2 */
	const uint16_t acl_interval_in_proc_interval_units = CONN_INTERVAL * 2;
	uint16_t desired_procedure_interval = realtime_rd ? 5 : 10;
	uint16_t desired_max_procedure_length =
		acl_interval_in_proc_interval_units * (desired_procedure_interval - 1);

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = desired_max_procedure_length,
		.min_procedure_interval = desired_procedure_interval,
		.max_procedure_interval = desired_procedure_interval,
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

	err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
	if (err) {
		LOG_ERR("Failed to set procedure parameters (err %d)", err);
		return err;
	}

	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = 1,
	};

	err = bt_le_cs_procedure_enable(conn, &params);
	if (err) {
		LOG_ERR("Failed to enable CS procedures (err %d)", err);
		return err;
	}

	return 0;
}
