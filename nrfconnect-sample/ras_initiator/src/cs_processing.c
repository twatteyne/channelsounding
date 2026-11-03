/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <string.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#include "cs_processing.h"

#define CS_STEP_MODE_0			(0)
#define TONE_QI_OK_TONE_COUNT_THRESHOLD (15)

static struct cs_procedure_result result;

static bool is_tone_quality_ok(const uint16_t num_iqs[CS_DE_NUM_CHANNELS],
			       const uint8_t *channel_map)
{
	uint8_t ok_tones_count = 0;

	for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; ++i) {
		if (BT_LE_CS_CHANNEL_BIT_GET(channel_map, i + CHANNEL_INDEX_OFFSET) &&
		    num_iqs[i] >= 1) {
			ok_tones_count += 1;
		}
	}
	return (ok_tones_count >= TONE_QI_OK_TONE_COUNT_THRESHOLD);
}

static void cumulate_mean(float *avg, float new_value, uint16_t *N)
{
	float a = 1.0f / (*N);
	float b = 1.0f - a;
	*avg = a * new_value + b * (*avg);
}

/* ---------------------------------------------------------------------------
 * Per-step extraction
 * ------------------------------------------------------------------------- */

static void extract_pcts(cs_de_report_t *p_report, uint8_t channel_index,
			 uint8_t antenna_permutation_index,
			 struct bt_hci_le_cs_step_data_tone_info *local_tone_info,
			 struct bt_hci_le_cs_step_data_tone_info *remote_tone_info)
{
	for (uint8_t tone_index = 0; tone_index < p_report->n_ap; tone_index++) {
		int antenna_path = bt_le_cs_get_antenna_path(p_report->n_ap,
							     antenna_permutation_index, tone_index);
		if (antenna_path < 0) {
			LOG_WRN("Invalid antenna path.");
			return;
		}

		if (local_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH ||
		    remote_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
			return;
		}

		struct bt_le_cs_iq_sample local_iq =
			bt_le_cs_parse_pct(local_tone_info[tone_index].phase_correction_term);
		struct bt_le_cs_iq_sample remote_iq =
			bt_le_cs_parse_pct(remote_tone_info[tone_index].phase_correction_term);

		uint16_t *n_iqs = &result.n_iqs[antenna_path][channel_index];
		cs_de_iq_tones_t *iq_tones = &p_report->iq_tones[antenna_path];

		(*n_iqs)++;

		if (*n_iqs == 1) {
			iq_tones->i_local[channel_index] = local_iq.i;
			iq_tones->q_local[channel_index] = local_iq.q;
			iq_tones->i_remote[channel_index] = remote_iq.i;
			iq_tones->q_remote[channel_index] = remote_iq.q;
		} else {
			cumulate_mean(&iq_tones->i_local[channel_index], local_iq.i, n_iqs);
			cumulate_mean(&iq_tones->q_local[channel_index], local_iq.q, n_iqs);
			cumulate_mean(&iq_tones->i_remote[channel_index], remote_iq.i, n_iqs);
			cumulate_mean(&iq_tones->q_remote[channel_index], remote_iq.q, n_iqs);
		}
	}
}

static void extract_rtt_timings(cs_de_report_t *p_report,
				struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
				struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data)
{
	if (local_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    local_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
	    peer_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    peer_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE) {
		return;
	}

	if (p_report->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
		p_report->rtt_accumulated_half_ns +=
			local_rtt_data->toa_tod_initiator - peer_rtt_data->tod_toa_reflector;
	} else {
		p_report->rtt_accumulated_half_ns +=
			peer_rtt_data->toa_tod_initiator - local_rtt_data->tod_toa_reflector;
	}

	p_report->rtt_count++;
}

static void accumulate_rssi(uint8_t packet_quality_aa_check, uint8_t packet_rssi)
{
	if (packet_quality_aa_check != BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE) {
		return;
	}

	result.rssi_sum += (int8_t)packet_rssi;
	result.rssi_count++;
}

/* ---------------------------------------------------------------------------
 * RAS parser callbacks
 * ------------------------------------------------------------------------- */

static bool process_ranging_header(struct ras_ranging_header *ranging_header, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

	p_report->n_ap = MAX(1, ((ranging_header->antenna_paths_mask & BIT(0)) +
				 ((ranging_header->antenna_paths_mask & BIT(1)) >> 1) +
				 ((ranging_header->antenna_paths_mask & BIT(2)) >> 2) +
				 ((ranging_header->antenna_paths_mask & BIT(3)) >> 3)));
	return true;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

	if (local_step->mode == CS_STEP_MODE_0 &&
	    p_report->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
		struct bt_hci_le_cs_step_data_mode_0_initiator *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_0_initiator *)local_step->data;

		accumulate_rssi(local_step_data->packet_quality_aa_check,
				local_step_data->packet_rssi);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index, local_step_data->tone_info,
			     peer_step_data->tone_info);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1) {
		struct bt_hci_le_cs_step_data_mode_1 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_1 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data;

		accumulate_rssi(local_step_data->packet_quality_aa_check,
				local_step_data->packet_rssi);
		extract_rtt_timings(p_report, local_step_data, peer_step_data);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
		struct bt_hci_le_cs_step_data_mode_3 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_3 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index, local_step_data->tone_info,
			     peer_step_data->tone_info);

		accumulate_rssi(local_step_data->packet_quality_aa_check,
				local_step_data->packet_rssi);
		extract_rtt_timings(p_report,
				    (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
				    (struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);
	}

	return true;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void cs_processing_extract(struct net_buf_simple *peer_steps,
			   struct net_buf_simple *local_steps,
			   const struct bt_conn_le_cs_config *config)
{
	cs_de_report_t *report = &result.report;

	memset(&result, 0, sizeof(result));
	report->role = config->role;

	bt_ras_rreq_rd_subevent_data_parse(peer_steps, local_steps, config->role,
					   process_ranging_header, NULL, process_step_data,
					   report);

	for (uint8_t ap = 0; ap < report->n_ap; ap++) {
		report->distance_estimates[ap].ifft = NAN;
		report->distance_estimates[ap].phase_slope = NAN;
		report->distance_estimates[ap].rtt = NAN;
		report->distance_estimates[ap].best = NAN;

		if (is_tone_quality_ok(result.n_iqs[ap], config->channel_map)) {
			report->tone_quality[ap] = CS_DE_TONE_QUALITY_OK;
		} else {
			report->tone_quality[ap] = CS_DE_TONE_QUALITY_BAD;
		}
	}
}

cs_de_quality_t cs_processing_estimate(void)
{
	return cs_de_calc(&result.report);
}

const struct cs_procedure_result *cs_processing_result(void)
{
	return &result;
}
