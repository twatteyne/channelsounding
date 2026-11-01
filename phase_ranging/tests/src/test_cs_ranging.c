/*
 * Unit tests of lib/cs_ranging.c + Nordic cs_de, on native_sim.
 *
 * Synthetic mode-2 steps are generated for a known distance:
 *   PCT_local  = A * exp(j*(+phi_lo - 2*pi*f*d/c))
 *   PCT_remote = A * exp(j*(-phi_lo - 2*pi*f*d/c))
 * with a random oscillator phase phi_lo per step, optional noise and
 * multipath, quantised to the 12-bit HCI PCT format.
 */

#include <unity.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/sys/printk.h>

#include "cs_ranging.h"

extern int unity_main(void);

#define PI_F          3.14159265358979f
#define C_M_S         299792458.0f
#define BASE_FREQ_HZ  2402e6f
#define SPACING_HZ    1e6f
#define AMPLITUDE     1000.0f
#define MIN_CHANNELS  15
#define TWO_PI_D      6.283185307179586

/* This Unity version has no TEST_ASSERT_LESS_THAN_FLOAT. */
#define ASSERT_BELOW(limit, value) TEST_ASSERT_TRUE_MESSAGE((value) < (limit), #value)

struct path {
	float distance_m;
	float gain;
};

/* ---- deterministic random generator ------------------------------------ */

static uint32_t rng_state;

static float rng_uniform(void)
{
	rng_state = rng_state * 1664525u + 1013904223u;
	return ((rng_state >> 8) + 0.5f) / 16777216.0f;
}

static float rng_gauss(void)
{
	float u1 = rng_uniform();
	float u2 = rng_uniform();

	return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}

/* ---- step generation ---------------------------------------------------- */

static bool channel_valid(int ch)
{
	return ch >= 2 && ch <= 76 && !(ch >= 23 && ch <= 25);
}

static void encode_pct(uint8_t pct[3], float i, float q)
{
	int32_t ii = CLAMP((int32_t)lrintf(i), -2048, 2047);
	int32_t qq = CLAMP((int32_t)lrintf(q), -2048, 2047);
	uint32_t v = ((uint32_t)ii & 0xFFF) | (((uint32_t)qq & 0xFFF) << 12);

	pct[0] = v & 0xFF;
	pct[1] = (v >> 8) & 0xFF;
	pct[2] = (v >> 16) & 0xFF;
}

/* Mode-2 step data for 1 antenna path: permutation index + 2 tone slots. */
struct step_buf {
	uint8_t raw[1 + 2 * sizeof(struct bt_hci_le_cs_step_data_tone_info)];
};

static struct bt_hci_le_cs_step_data_mode_2 *step_init(struct step_buf *b)
{
	memset(b, 0, sizeof(*b));
	return (struct bt_hci_le_cs_step_data_mode_2 *)b->raw;
}

/* Add one simulated mode-2 step on @p ch. noise_std is relative to AMPLITUDE. */
static void add_step(struct cs_ranging_proc *proc, int ch, const struct path *paths, int n_paths,
		     float noise_std, bool good_quality)
{
	struct step_buf lb, rb;
	struct bt_hci_le_cs_step_data_mode_2 *l = step_init(&lb);
	struct bt_hci_le_cs_step_data_mode_2 *r = step_init(&rb);
	float f = BASE_FREQ_HZ + ch * SPACING_HZ;
	float phi_lo = 2.0f * PI_F * rng_uniform();

	/* One-way channel response h = sum g * exp(-j*2*pi*f*d/c). */
	float h_re = 0.0f, h_im = 0.0f;

	for (int p = 0; p < n_paths; p++) {
		/* Phase in double precision: f*d/c is large. */
		double ph = -TWO_PI_D * (double)f * (double)paths[p].distance_m / (double)C_M_S;
		h_re += paths[p].gain * (float)cos(fmod(ph, TWO_PI_D));
		h_im += paths[p].gain * (float)sin(fmod(ph, TWO_PI_D));
	}

	float lo_re = cosf(phi_lo), lo_im = sinf(phi_lo);
	/* local = A*h*e^{+j phi_lo}, remote = A*h*e^{-j phi_lo} */
	float li = AMPLITUDE * (h_re * lo_re - h_im * lo_im);
	float lq = AMPLITUDE * (h_re * lo_im + h_im * lo_re);
	float ri = AMPLITUDE * (h_re * lo_re + h_im * lo_im);
	float rq = AMPLITUDE * (h_im * lo_re - h_re * lo_im);

	li += AMPLITUDE * noise_std * rng_gauss();
	lq += AMPLITUDE * noise_std * rng_gauss();
	ri += AMPLITUDE * noise_std * rng_gauss();
	rq += AMPLITUDE * noise_std * rng_gauss();

	encode_pct(l->tone_info[0].phase_correction_term, li, lq);
	encode_pct(r->tone_info[0].phase_correction_term, ri, rq);
	l->tone_info[0].quality_indicator = BT_HCI_LE_CS_TONE_QUALITY_HIGH;
	r->tone_info[0].quality_indicator =
		good_quality ? BT_HCI_LE_CS_TONE_QUALITY_HIGH : BT_HCI_LE_CS_TONE_QUALITY_LOW;

	cs_ranging_add_mode2_step(proc, (uint8_t)ch, l, r);
}

/* Simulate a whole procedure: every channel kept with probability keep, @p reps steps each. */
static void simulate(struct cs_ranging_proc *proc, const struct path *paths, int n_paths,
		     float noise_std, int reps, float keep)
{
	cs_ranging_reset(proc, BT_CONN_LE_CS_ROLE_INITIATOR, 1);

	for (int ch = 0; ch <= 78; ch++) {
		if (!channel_valid(ch) || rng_uniform() >= keep) {
			continue;
		}
		for (int k = 0; k < reps; k++) {
			add_step(proc, ch, paths, n_paths, noise_std, true);
		}
	}
}

static struct cs_ranging_proc proc;
static struct cs_ranging_estimate est[CS_RANGING_MAX_AP];

void setUp(void)
{
	rng_state = 12345u;
}

void tearDown(void)
{
}

/* ---- tests -------------------------------------------------------------- */

void test_line_of_sight_noiseless(void)
{
	float max_err = 0.0f;

	/* cs_de phase slope: atan2 in (-pi, pi] => 0..75 m (negative => NAN). */
	for (float d = 0.5f; d < 74.5f; d += 0.7f) {
		struct path p = {d, 1.0f};

		simulate(&proc, &p, 1, 0.0f, 1, 1.1f);
		TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
		TEST_ASSERT_EQUAL_UINT8(72, est[0].n_channels);
		TEST_ASSERT_FLOAT_WITHIN(0.05f, d, est[0].ifft_m);
		TEST_ASSERT_FLOAT_WITHIN(0.05f, d, est[0].phase_slope_m);
		TEST_ASSERT_EQUAL_FLOAT(est[0].ifft_m, est[0].best_m);
		max_err = fmaxf(max_err, fmaxf(fabsf(est[0].ifft_m - d),
					       fabsf(est[0].phase_slope_m - d)));
	}
	printk("Line of sight 0.5..74 m, no noise: max error %.3f m\n", (double)max_err);
}

void test_range_limits(void)
{
	/* Beyond 75 m only the IFFT is available (up to ~149 m). */
	struct path p = {100.0f, 1.0f};

	simulate(&proc, &p, 1, 0.0f, 1, 1.1f);
	TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
	TEST_ASSERT_TRUE(isnan(est[0].phase_slope_m));
	TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, est[0].ifft_m);
	TEST_ASSERT_EQUAL_FLOAT(est[0].ifft_m, est[0].best_m);

	/* 160 m folds back to 160 - 149.9 = 10.1 m (1 MHz spacing ambiguity). */
	p.distance_m = 160.0f;
	simulate(&proc, &p, 1, 0.0f, 1, 1.1f);
	TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
	TEST_ASSERT_FLOAT_WITHIN(0.05f, 160.0f - 149.896f, est[0].ifft_m);
}

static float rms_error(float d, float noise_std, int reps, float keep, int runs, bool slope)
{
	struct path p = {d, 1.0f};
	float se = 0.0f;

	for (int i = 0; i < runs; i++) {
		simulate(&proc, &p, 1, noise_std, reps, keep);
		TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
		float e = (slope ? est[0].phase_slope_m : est[0].ifft_m) - d;

		se += e * e;
	}
	return sqrtf(se / runs);
}

void test_noise_20db(void)
{
	/* noise_std 0.07 per I/Q component on each PCT ~ 20 dB SNR */
	float e_ifft = rms_error(7.3f, 0.07f, 1, 1.1f, 50, false);
	float e_slope = rms_error(7.3f, 0.07f, 1, 1.1f, 50, true);

	printk("SNR 20 dB, 7.3 m: RMS error ifft %.3f m, phase slope %.3f m\n", (double)e_ifft,
	       (double)e_slope);
	ASSERT_BELOW(0.15f, e_ifft);
	ASSERT_BELOW(0.15f, e_slope);
}

void test_repetitions_average_product(void)
{
	/* Each repetition has its own random LO phase: only the averaged
	 * local*remote product is coherent. 4 repetitions must reduce the error.
	 */
	float e1 = rms_error(7.3f, 0.2f, 1, 1.1f, 50, false);
	float e4 = rms_error(7.3f, 0.2f, 4, 1.1f, 50, false);

	printk("SNR 10 dB, 7.3 m: RMS error ifft %.3f m (1 rep), %.3f m (4 reps)\n", (double)e1,
	       (double)e4);
	ASSERT_BELOW(e1, e4);
	ASSERT_BELOW(0.2f, e4);
}

void test_missing_channels(void)
{
	struct path p = {12.0f, 1.0f};

	simulate(&proc, &p, 1, 0.02f, 1, 0.6f);
	TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
	TEST_ASSERT_LESS_THAN_UINT8(72, est[0].n_channels);
	TEST_ASSERT_GREATER_OR_EQUAL_UINT8(MIN_CHANNELS, est[0].n_channels);
	TEST_ASSERT_FLOAT_WITHIN(0.3f, 12.0f, est[0].ifft_m);
}

void test_too_few_channels(void)
{
	struct path p = {5.0f, 1.0f};

	cs_ranging_reset(&proc, BT_CONN_LE_CS_ROLE_INITIATOR, 1);
	for (int ch = 2; ch < 2 + MIN_CHANNELS - 1; ch++) {
		add_step(&proc, ch, &p, 1, 0.0f, true);
	}
	TEST_ASSERT_EQUAL_INT(0, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
	TEST_ASSERT_FALSE(est[0].valid);
	TEST_ASSERT_TRUE(isnan(est[0].best_m));
}

void test_low_quality_tones_rejected(void)
{
	struct path p = {5.0f, 1.0f};

	cs_ranging_reset(&proc, BT_CONN_LE_CS_ROLE_INITIATOR, 1);
	for (int ch = 0; ch <= 78; ch++) {
		if (channel_valid(ch)) {
			add_step(&proc, ch, &p, 1, 0.0f, false);
		}
	}
	TEST_ASSERT_EQUAL_UINT32(0, proc.tones_ok);
	TEST_ASSERT_EQUAL_UINT32(72, proc.tones_rejected);
	TEST_ASSERT_EQUAL_INT(0, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
}

void test_invalid_channels_rejected(void)
{
	struct bt_le_cs_iq_sample one = {.i = 100, .q = 0};

	cs_ranging_reset(&proc, BT_CONN_LE_CS_ROLE_INITIATOR, 1);
	TEST_ASSERT_EQUAL_INT(-EINVAL, cs_ranging_add_tone(&proc, 1, 0, one, one));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cs_ranging_add_tone(&proc, 24, 0, one, one));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cs_ranging_add_tone(&proc, 77, 0, one, one));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cs_ranging_add_tone(&proc, 10, 1, one, one));
	TEST_ASSERT_EQUAL_INT(0, cs_ranging_add_tone(&proc, 76, 0, one, one));
	TEST_ASSERT_EQUAL_UINT8(1, cs_ranging_channel_count(&proc, 0));
}

void test_calibration_offset(void)
{
	struct path p = {5.0f + 1.2f, 1.0f};

	simulate(&proc, &p, 1, 0.0f, 1, 1.1f);
	TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 1.2f, est));
	TEST_ASSERT_FLOAT_WITHIN(0.05f, 5.0f, est[0].best_m);
	TEST_ASSERT_FLOAT_WITHIN(0.05f, 5.0f, est[0].phase_slope_m);
}

/*
 * Multipath. The channel is reciprocal, so the product of the two PCTs is
 * h^2 = sum_p sum_q g_p g_q exp(-j*2*pi*f*(d_p + d_q)/c): besides the two
 * paths, the IFFT shows a cross term at (d_1 + d_2) / 2 with gain 2*g_1*g_2.
 * cs_de keeps the first peak above 40 % of the maximum.
 */
void test_multipath_ifft_finds_direct_path(void)
{
	/* Direct 4 m (1.0) + reflection 13 m (0.6): terms 4 m 1.0, 8.5 m 1.2, 13 m 0.36. */
	const struct path p[] = {{4.0f, 1.0f}, {13.0f, 0.6f}};

	simulate(&proc, p, 2, 0.0f, 1, 1.1f);
	TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
	printk("Multipath 4 m (1.0) + 13 m (0.6): ifft %.2f m, phase slope %.2f m\n",
	       (double)est[0].ifft_m, (double)est[0].phase_slope_m);
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 4.0f, est[0].ifft_m);
	/* The phase slope is pulled away from the direct path. */
	TEST_ASSERT_TRUE(fabsf(est[0].phase_slope_m - 4.0f) > 1.0f);
	TEST_ASSERT_EQUAL_FLOAT(est[0].ifft_m, est[0].best_m);
}

void test_multipath_weak_direct_path_limitation(void)
{
	/* Direct 4 m (0.6) + reflection 13 m (1.0): terms 4 m 0.36, 8.5 m 1.2, 13 m 1.0.
	 * The direct term is below 40 % of the cross term: known limitation,
	 * cs_de returns the cross term (~8.5 m).
	 */
	const struct path p[] = {{4.0f, 0.6f}, {13.0f, 1.0f}};

	simulate(&proc, p, 2, 0.0f, 1, 1.1f);
	TEST_ASSERT_EQUAL_INT(1, cs_ranging_compute(&proc, MIN_CHANNELS, 0.0f, est));
	printk("Multipath 4 m (0.6) + 13 m (1.0): ifft %.2f m, phase slope %.2f m\n",
	       (double)est[0].ifft_m, (double)est[0].phase_slope_m);
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 8.5f, est[0].ifft_m);
}

void test_median(void)
{
	struct cs_ranging_median m;

	cs_ranging_median_init(&m, 3);
	TEST_ASSERT_TRUE(isnan(cs_ranging_median_push(&m, NAN)));
	TEST_ASSERT_EQUAL_FLOAT(5.0f, cs_ranging_median_push(&m, 5.0f));
	TEST_ASSERT_EQUAL_FLOAT(3.0f, cs_ranging_median_push(&m, 1.0f));
	TEST_ASSERT_EQUAL_FLOAT(5.0f, cs_ranging_median_push(&m, 9.0f));
	TEST_ASSERT_EQUAL_FLOAT(5.0f, cs_ranging_median_push(&m, NAN));
	/* The oldest value (5) leaves the window: {1, 9, 100}. */
	TEST_ASSERT_EQUAL_FLOAT(9.0f, cs_ranging_median_push(&m, 100.0f));
	TEST_ASSERT_EQUAL_UINT8(3, m.len);
}

int main(void)
{
	(void)unity_main();
	return 0;
}
