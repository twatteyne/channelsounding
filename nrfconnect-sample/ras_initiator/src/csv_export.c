/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "app_config.h"
#include "csv_export.h"

#define CSV_HEADER                                                                                 \
	"timestamp_ms,ranging_counter,ap,channel,freq_mhz,h_re,h_im,amplitude,phase_deg,n_iqs"

#define CSV_PROC_HEADER                                                                            \
	"timestamp_ms,ranging_counter,ap,tone_quality,rtt_count,rtt_mean_ns,tof_ns,rssi_dbm,"     \
	"dist_ifft_m,dist_phase_slope_m,dist_rtt_m"

#define CSV_THREAD_STACK_SIZE 2048
#define CSV_THREAD_PRIORITY   K_LOWEST_APPLICATION_THREAD_PRIO

struct csv_snapshot {
	int64_t timestamp_ms;
	uint16_t ranging_counter;
	uint8_t n_ap;
	cs_de_iq_tones_t iq_tones[MAX_AP];
	uint16_t n_iqs[MAX_AP][CS_DE_NUM_CHANNELS];
	cs_de_tone_quality_t tone_quality[MAX_AP];
	cs_de_dist_estimates_t distance_estimates[MAX_AP];
	int32_t rtt_accumulated_half_ns;
	uint8_t rtt_count;
	float rssi_dbm;
};

static struct csv_snapshot csv_snapshot;
static atomic_t csv_snapshot_busy;
static K_SEM_DEFINE(sem_csv_ready, 0, 1);
static uint32_t csv_skipped_procedures;

void csv_export_submit(uint16_t ranging_counter, const struct cs_procedure_result *res)
{
	const cs_de_report_t *report = &res->report;

	if (!atomic_cas(&csv_snapshot_busy, 0, 1)) {
		/* Previous procedure still being printed: skip this one. */
		csv_skipped_procedures++;
		return;
	}

	csv_snapshot.timestamp_ms = k_uptime_get();
	csv_snapshot.ranging_counter = ranging_counter;
	csv_snapshot.n_ap = MIN(report->n_ap, MAX_AP);
	memcpy(csv_snapshot.iq_tones, report->iq_tones, sizeof(csv_snapshot.iq_tones));
	memcpy(csv_snapshot.n_iqs, res->n_iqs, sizeof(csv_snapshot.n_iqs));
	memcpy(csv_snapshot.tone_quality, report->tone_quality,
	       sizeof(csv_snapshot.tone_quality));
	memcpy(csv_snapshot.distance_estimates, report->distance_estimates,
	       sizeof(csv_snapshot.distance_estimates));
	csv_snapshot.rtt_accumulated_half_ns = report->rtt_accumulated_half_ns;
	csv_snapshot.rtt_count = report->rtt_count;
	csv_snapshot.rssi_dbm = res->rssi_count ? (float)res->rssi_sum / res->rssi_count : NAN;

	k_sem_give(&sem_csv_ready);
}

static void csv_snapshot_print(const struct csv_snapshot *snap)
{
	/* RTT (and therefore ToF / RTT distance) is computed once per procedure. */
	float rtt_mean_ns = NAN;
	float tof_ns = NAN;

	if (snap->rtt_count > 0) {
		rtt_mean_ns = (snap->rtt_accumulated_half_ns * 0.5f) / snap->rtt_count;
		tof_ns = rtt_mean_ns / 2.0f;
	}

	float dist_rtt_m = snap->distance_estimates[0].rtt;

	for (uint8_t ap = 0; ap < snap->n_ap; ap++) {
		const cs_de_iq_tones_t *iq = &snap->iq_tones[ap];
		const cs_de_dist_estimates_t *de = &snap->distance_estimates[ap];
		const char *tq = snap->tone_quality[ap] == CS_DE_TONE_QUALITY_OK ? "OK" : "BAD";

		printk("CSVP,%lld,%u,%u,%s,%u,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f\n", snap->timestamp_ms,
		       snap->ranging_counter, ap, tq, snap->rtt_count, (double)rtt_mean_ns,
		       (double)tof_ns, (double)snap->rssi_dbm, (double)de->ifft,
		       (double)de->phase_slope, (double)dist_rtt_m);

		for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; i++) {
			if (snap->n_iqs[ap][i] == 0) {
				continue;
			}

			/* Two-way channel response H = local * remote (same as cs_de). */
			float h_re = iq->i_local[i] * iq->i_remote[i] -
				     iq->q_local[i] * iq->q_remote[i];
			float h_im = iq->i_local[i] * iq->q_remote[i] +
				     iq->i_remote[i] * iq->q_local[i];
			float amplitude = sqrtf(h_re * h_re + h_im * h_im);
			float phase_deg = atan2f(h_im, h_re) * (180.0f / 3.14159265f);
			uint8_t channel = i + CHANNEL_INDEX_OFFSET;

			printk("CSV,%lld,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.2f,%u\n", snap->timestamp_ms,
			       snap->ranging_counter, ap, channel, 2402 + channel, (double)h_re,
			       (double)h_im, (double)amplitude, (double)phase_deg,
			       snap->n_iqs[ap][i]);
		}
	}
}

static void csv_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("CSVH," CSV_HEADER "\n");
	printk("CSVPH," CSV_PROC_HEADER "\n");

	while (true) {
		k_sem_take(&sem_csv_ready, K_FOREVER);
		csv_snapshot_print(&csv_snapshot);
		atomic_set(&csv_snapshot_busy, 0);
	}
}

K_THREAD_DEFINE(csv_thread_id, CSV_THREAD_STACK_SIZE, csv_thread, NULL, NULL, NULL,
		CSV_THREAD_PRIORITY, 0, 0);
