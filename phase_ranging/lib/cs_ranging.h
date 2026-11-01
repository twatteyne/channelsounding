/*
 * Phase-Based Ranging on top of the Nordic cs_de library.
 *
 * The distance estimation itself (phase slope and IFFT) is done by the NCS
 * Channel Sounding Distance Estimation library (<bluetooth/cs_de.h>,
 * CONFIG_BT_CS_DE). This module only does what cs_de leaves to the
 * application:
 *
 *   1. Turn the CS mode-2 step data (local + remote PCTs) of one procedure
 *      into a cs_de_report_t: quality filtering, antenna-path mapping,
 *      per-channel averaging.
 *   2. Decide per antenna path whether there are enough usable channels
 *      (cs_de_report_t.tone_quality).
 *   3. Call cs_de_calc(), then subtract a calibration offset.
 *   4. Smooth the successive estimates with a sliding median.
 *
 * Averaging note: when a channel is measured several times in a procedure,
 * the local x remote product (which cancels the oscillator phase offset of
 * each step) is averaged, not the local and remote PCTs separately. The
 * averaged product is then stored in i_local/q_local, with i_remote = 1 and
 * q_remote = 0, so that cs_de_combined_iq_calculate() returns it unchanged.
 */

#ifndef CS_RANGING_H_
#define CS_RANGING_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/hci_types.h>
#include <bluetooth/cs_de.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CS channel of cs_de index 0 (2404 MHz). */
#define CS_RANGING_FIRST_CHANNEL 2

/** Maximum number of antenna paths handled (same as cs_de). */
#define CS_RANGING_MAX_AP CONFIG_BT_CS_DE_MAX_NUM_ANTENNA_PATHS

/** Data of one CS procedure, filled step by step. */
struct cs_ranging_proc {
	/** Report handed to cs_de_calc(). */
	cs_de_report_t report;
	/** Number of good tones accumulated per antenna path and channel. */
	uint16_t n_iq[CS_RANGING_MAX_AP][CS_DE_NUM_CHANNELS];
	/** Tones accepted / rejected (quality indicator not HIGH on either side). */
	uint32_t tones_ok;
	uint32_t tones_rejected;
};

/** Result of one procedure for one antenna path, in metres (offset already removed). */
struct cs_ranging_estimate {
	/** cs_de IFFT estimate (NAN if not available). */
	float ifft_m;
	/** cs_de phase-slope estimate (NAN if not available). */
	float phase_slope_m;
	/** cs_de "best" estimate (NAN if not available). */
	float best_m;
	/** Number of channels that hold at least one good tone. */
	uint8_t n_channels;
	/** true if cs_de produced a usable estimate for this path. */
	bool valid;
};

/** Sliding median over the last estimates. */
#define CS_RANGING_MEDIAN_MAX 32

struct cs_ranging_median {
	float values[CS_RANGING_MEDIAN_MAX];
	uint8_t size;
	uint8_t len;
	uint8_t idx;
};

/**
 * Start a new procedure.
 *
 * @param role CS role of this device (stored in the report).
 * @param n_ap Number of antenna paths (1..CS_RANGING_MAX_AP, clamped).
 */
void cs_ranging_reset(struct cs_ranging_proc *proc, enum bt_conn_le_cs_role role, uint8_t n_ap);

/**
 * Add one tone measured on a CS channel, already parsed as IQ.
 *
 * @param channel CS channel (2..76, except 23..25).
 * @param ap Antenna path (0..n_ap-1).
 * @return 0 on success, -EINVAL if the channel or path is invalid.
 */
int cs_ranging_add_tone(struct cs_ranging_proc *proc, uint8_t channel, uint8_t ap,
			struct bt_le_cs_iq_sample local, struct bt_le_cs_iq_sample remote);

/**
 * Add the tones of one mode-2 step (local and peer step data of the same step).
 *
 * Tones whose quality indicator is not HIGH on both sides are rejected. The
 * tone index is mapped to the antenna path with bt_le_cs_get_antenna_path().
 */
void cs_ranging_add_mode2_step(struct cs_ranging_proc *proc, uint8_t channel,
			       const struct bt_hci_le_cs_step_data_mode_2 *local,
			       const struct bt_hci_le_cs_step_data_mode_2 *peer);

/** Number of channels with at least one good tone on antenna path @p ap. */
uint8_t cs_ranging_channel_count(const struct cs_ranging_proc *proc, uint8_t ap);

/**
 * Run cs_de on the accumulated procedure.
 *
 * A path is given to cs_de only if it has at least @p min_channels channels
 * (otherwise its tone_quality is set to CS_DE_TONE_QUALITY_BAD).
 * @p offset_m is subtracted from every finite estimate.
 *
 * Note: cs_de_calc() uses a static scratch buffer, so this function is not
 * reentrant.
 *
 * @param out Array of CS_RANGING_MAX_AP results.
 * @return Number of antenna paths with a valid estimate.
 */
int cs_ranging_compute(struct cs_ranging_proc *proc, uint8_t min_channels, float offset_m,
		       struct cs_ranging_estimate out[CS_RANGING_MAX_AP]);

/** Initialise a sliding median of @p size values (1..CS_RANGING_MEDIAN_MAX, clamped). */
void cs_ranging_median_init(struct cs_ranging_median *m, uint8_t size);

/** Push a value (ignored if not finite) and return the current median (NAN if empty). */
float cs_ranging_median_push(struct cs_ranging_median *m, float value);

#ifdef __cplusplus
}
#endif

#endif /* CS_RANGING_H_ */
