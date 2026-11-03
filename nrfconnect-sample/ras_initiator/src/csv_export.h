/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief CSV export of per-procedure CS data over the console.
 *
 *  Two record types, so that per-procedure values are not repeated on every
 *  channel row:
 *   - tones:      one row per (procedure, antenna path, channel with at least
 *                 one valid tone), prefixed with "CSV," (header "CSVH,").
 *   - procedures: one row per (procedure, antenna path), prefixed with "CSVP,"
 *                 (header "CSVPH,"). Holds everything computed once per
 *                 procedure: tone quality, RTT, ToF, RSSI, distance estimates.
 *  Join the two on (ranging_counter, ap).
 *  Must match TONE_HEADER / PROC_HEADER in scripts/cs_csv_logger.py.
 *
 *  Printing is done by a low-priority thread so that the Bluetooth thread is
 *  never blocked by the console.
 */

#ifndef CSV_EXPORT_H_
#define CSV_EXPORT_H_

#include <stdint.h>

#include "cs_processing.h"

/** Copy the procedure result so it can be printed by the CSV thread.
 *
 *  If the previous procedure is still being printed, this one is skipped.
 */
void csv_export_submit(uint16_t ranging_counter, const struct cs_procedure_result *res);

#endif /* CSV_EXPORT_H_ */
