/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#include "app_config.h"
#include "distance_estimate.h"

#define DE_SLIDING_WINDOW_SIZE (9)

struct distance_estimate_buffer {
	cs_de_dist_estimates_t estimates[DE_SLIDING_WINDOW_SIZE];
	uint8_t num_valid;
	uint8_t index;
};

static struct distance_estimate_buffer distance_estimate_buffers[MAX_AP];

static K_MUTEX_DEFINE(distance_estimate_buffer_mutex);
static K_SEM_DEFINE(sem_distance_estimate_updated, 0, 1);

static void store_distance_estimates_in_buffer(const cs_de_dist_estimates_t *p_estimates,
					       struct distance_estimate_buffer *buffer)
{
	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	memcpy(&buffer->estimates[buffer->index], p_estimates, sizeof(cs_de_dist_estimates_t));

	buffer->index = (buffer->index + 1) % DE_SLIDING_WINDOW_SIZE;

	if (buffer->num_valid < DE_SLIDING_WINDOW_SIZE) {
		buffer->num_valid++;
	}

	k_mutex_unlock(&distance_estimate_buffer_mutex);
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
	if (count == 0) {
		return NAN;
	}

	qsort(values, count, sizeof(float), float_cmp);

	if (count % 2 == 0) {
		return (values[count/2] + values[count/2 - 1]) / 2;
	} else {
		return values[count/2];
	}
}

static cs_de_dist_estimates_t get_distance(uint8_t ap)
{
	cs_de_dist_estimates_t averaged_result = {};
	uint8_t num_ifft = 0;
	uint8_t num_phase_slope = 0;
	uint8_t num_rtt = 0;

	static float temp_ifft[DE_SLIDING_WINDOW_SIZE];
	static float temp_phase_slope[DE_SLIDING_WINDOW_SIZE];
	static float temp_rtt[DE_SLIDING_WINDOW_SIZE];

	struct distance_estimate_buffer *buffer = &distance_estimate_buffers[ap];

	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	for (uint8_t i = 0; i < buffer->num_valid; i++) {
		if (isfinite(buffer->estimates[i].ifft)) {
			temp_ifft[num_ifft] = buffer->estimates[i].ifft;
			num_ifft++;
		}
		if (isfinite(buffer->estimates[i].phase_slope)) {
			temp_phase_slope[num_phase_slope] = buffer->estimates[i].phase_slope;
			num_phase_slope++;
		}
		if (isfinite(buffer->estimates[i].rtt)) {
			temp_rtt[num_rtt] = buffer->estimates[i].rtt;
			num_rtt++;
		}
	}

	k_mutex_unlock(&distance_estimate_buffer_mutex);

	averaged_result.ifft = median_inplace(num_ifft, temp_ifft);
	averaged_result.phase_slope = median_inplace(num_phase_slope, temp_phase_slope);
	averaged_result.rtt = median_inplace(num_rtt, temp_rtt);

	return averaged_result;
}

void distance_estimate_update(const cs_de_report_t *report)
{
	for (uint8_t ap = 0; ap < report->n_ap; ap++) {
		if (report->tone_quality[ap] == CS_DE_TONE_QUALITY_OK ||
		    isfinite(report->distance_estimates[ap].rtt)) {
			store_distance_estimates_in_buffer(&report->distance_estimates[ap],
							   &distance_estimate_buffers[ap]);
		}
	}
	k_sem_give(&sem_distance_estimate_updated);
}

void distance_estimate_wait(void)
{
	k_sem_take(&sem_distance_estimate_updated, K_FOREVER);
}

bool distance_estimate_available(uint8_t ap)
{
	return distance_estimate_buffers[ap].num_valid != 0;
}

void distance_estimate_print(uint8_t ap)
{
	cs_de_dist_estimates_t distance_on_ap = get_distance(ap);
#if defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_1)
	LOG_INF("Latest distance estimate rtt: %.2f meters", (double)distance_on_ap.rtt);
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2)
	LOG_INF("Latest distance estimates on antenna path %u: ifft: %.2f, phase_slope: %.2f "
		"meters",
		ap, (double)distance_on_ap.ifft, (double)distance_on_ap.phase_slope);
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2_SUB_MODE_1) || \
	defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_3)
	LOG_INF("Latest distance estimates on antenna path %u: ifft: %.2f, phase_slope: %.2f, rtt: "
		"%.2f meters",
		ap, (double)distance_on_ap.ifft, (double)distance_on_ap.phase_slope,
		(double)distance_on_ap.rtt);
#else
	BUILD_ASSERT(false, "Invalid ranging mode");
#endif
}
