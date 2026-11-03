/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Sliding-window median filtering of the distance estimates.
 */

#ifndef DISTANCE_ESTIMATE_H_
#define DISTANCE_ESTIMATE_H_

#include <stdbool.h>
#include <stdint.h>
#include <bluetooth/cs_de.h>

/** Store the estimates of every usable antenna path of a report and wake up the reader. */
void distance_estimate_update(const cs_de_report_t *report);

/** Block until new estimates have been stored. */
void distance_estimate_wait(void);

/** True if at least one estimate has been stored for the antenna path. */
bool distance_estimate_available(uint8_t ap);

/** Log the median of the sliding window for the antenna path. */
void distance_estimate_print(uint8_t ap);

#endif /* DISTANCE_ESTIMATE_H_ */
