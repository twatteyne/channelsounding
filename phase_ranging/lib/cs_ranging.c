/*
 * Phase-Based Ranging on top of the Nordic cs_de library. See cs_ranging.h.
 */

#include "cs_ranging.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/util.h>

static bool channel_to_index(uint8_t channel, uint8_t *index)
{
	if (channel < CS_RANGING_FIRST_CHANNEL ||
	    channel >= CS_RANGING_FIRST_CHANNEL + CS_DE_NUM_CHANNELS ||
	    (channel >= 23 && channel <= 25)) {
		return false;
	}

	*index = channel - CS_RANGING_FIRST_CHANNEL;
	return true;
}

void cs_ranging_reset(struct cs_ranging_proc *proc, enum bt_conn_le_cs_role role, uint8_t n_ap)
{
	memset(proc, 0, sizeof(*proc));

	proc->report.role = role;
	proc->report.n_ap = CLAMP(n_ap, 1, CS_RANGING_MAX_AP);

	for (int ap = 0; ap < CS_RANGING_MAX_AP; ap++) {
		/* Neutral remote phasor: cs_de multiplies local by remote. */
		for (int n = 0; n < CS_DE_NUM_CHANNELS; n++) {
			proc->report.iq_tones[ap].i_remote[n] = 1.0f;
		}
		proc->report.distance_estimates[ap].ifft = NAN;
		proc->report.distance_estimates[ap].phase_slope = NAN;
		proc->report.distance_estimates[ap].rtt = NAN;
		proc->report.distance_estimates[ap].best = NAN;
		proc->report.tone_quality[ap] = CS_DE_TONE_QUALITY_BAD;
	}
}

int cs_ranging_add_tone(struct cs_ranging_proc *proc, uint8_t channel, uint8_t ap,
			struct bt_le_cs_iq_sample local, struct bt_le_cs_iq_sample remote)
{
	uint8_t n;

	if (ap >= proc->report.n_ap || !channel_to_index(channel, &n)) {
		return -EINVAL;
	}

	/* H = PCT_local * PCT_remote: the oscillator phase offset cancels out. */
	float re = (float)local.i * remote.i - (float)local.q * remote.q;
	float im = (float)local.i * remote.q + (float)local.q * remote.i;

	cs_de_iq_tones_t *t = &proc->report.iq_tones[ap];
	uint16_t count = ++proc->n_iq[ap][n];

	/* Running mean of the product. */
	t->i_local[n] += (re - t->i_local[n]) / count;
	t->q_local[n] += (im - t->q_local[n]) / count;

	return 0;
}

void cs_ranging_add_mode2_step(struct cs_ranging_proc *proc, uint8_t channel,
			       const struct bt_hci_le_cs_step_data_mode_2 *local,
			       const struct bt_hci_le_cs_step_data_mode_2 *peer)
{
	const uint8_t n_ap = proc->report.n_ap;

	/* One tone per antenna path (the extension slot, index n_ap, is not used). */
	for (uint8_t tone = 0; tone < n_ap; tone++) {
		const struct bt_hci_le_cs_step_data_tone_info *l = &local->tone_info[tone];
		const struct bt_hci_le_cs_step_data_tone_info *r = &peer->tone_info[tone];

		if (l->quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH ||
		    r->quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
			proc->tones_rejected++;
			continue;
		}

		int ap = bt_le_cs_get_antenna_path(n_ap, local->antenna_permutation_index, tone);

		if (ap < 0) {
			proc->tones_rejected++;
			continue;
		}

		if (cs_ranging_add_tone(proc, channel, (uint8_t)ap,
					bt_le_cs_parse_pct(l->phase_correction_term),
					bt_le_cs_parse_pct(r->phase_correction_term)) == 0) {
			proc->tones_ok++;
		} else {
			proc->tones_rejected++;
		}
	}
}

uint8_t cs_ranging_channel_count(const struct cs_ranging_proc *proc, uint8_t ap)
{
	uint8_t count = 0;

	if (ap >= CS_RANGING_MAX_AP) {
		return 0;
	}

	for (int n = 0; n < CS_DE_NUM_CHANNELS; n++) {
		if (proc->n_iq[ap][n] > 0) {
			count++;
		}
	}

	return count;
}

static float remove_offset(float d, float offset_m)
{
	return isfinite(d) ? d - offset_m : NAN;
}

int cs_ranging_compute(struct cs_ranging_proc *proc, uint8_t min_channels, float offset_m,
		       struct cs_ranging_estimate out[CS_RANGING_MAX_AP])
{
	cs_de_report_t *rep = &proc->report;
	int n_valid = 0;

	/* No RTT in mode 2 only: cs_de then skips that estimator. */
	rep->rtt_count = 0;
	rep->rtt_accumulated_half_ns = 0;

	for (uint8_t ap = 0; ap < CS_RANGING_MAX_AP; ap++) {
		out[ap] = (struct cs_ranging_estimate){
			.ifft_m = NAN, .phase_slope_m = NAN, .best_m = NAN,
			.n_channels = cs_ranging_channel_count(proc, ap), .valid = false,
		};

		if (ap < rep->n_ap) {
			rep->tone_quality[ap] = (out[ap].n_channels >= MAX(min_channels, 2))
							? CS_DE_TONE_QUALITY_OK
							: CS_DE_TONE_QUALITY_BAD;
		}
	}

	if (cs_de_calc(rep) != CS_DE_QUALITY_OK) {
		return 0;
	}

	for (uint8_t ap = 0; ap < rep->n_ap; ap++) {
		const cs_de_dist_estimates_t *e = &rep->distance_estimates[ap];

		/* cs_de leaves the estimates of BAD paths untouched (NAN from reset). */
		if (rep->tone_quality[ap] != CS_DE_TONE_QUALITY_OK) {
			continue;
		}

		out[ap].ifft_m = remove_offset(e->ifft, offset_m);
		out[ap].phase_slope_m = remove_offset(e->phase_slope, offset_m);
		/* cs_de sets best to 0 when nothing is finite: treat as invalid. */
		if (isfinite(e->ifft) || isfinite(e->phase_slope) || isfinite(e->rtt)) {
			out[ap].best_m = remove_offset(e->best, offset_m);
			out[ap].valid = true;
			n_valid++;
		}
	}

	return n_valid;
}

void cs_ranging_median_init(struct cs_ranging_median *m, uint8_t size)
{
	memset(m, 0, sizeof(*m));
	m->size = CLAMP(size, 1, CS_RANGING_MEDIAN_MAX);
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

float cs_ranging_median_push(struct cs_ranging_median *m, float value)
{
	float sorted[CS_RANGING_MEDIAN_MAX];

	if (isfinite(value)) {
		m->values[m->idx] = value;
		m->idx = (m->idx + 1) % m->size;
		if (m->len < m->size) {
			m->len++;
		}
	}

	if (m->len == 0) {
		return NAN;
	}

	memcpy(sorted, m->values, m->len * sizeof(float));
	qsort(sorted, m->len, sizeof(float), float_cmp);

	return (m->len % 2) ? sorted[m->len / 2]
			    : 0.5f * (sorted[m->len / 2 - 1] + sorted[m->len / 2]);
}
