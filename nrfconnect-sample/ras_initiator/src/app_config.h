/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Compile-time configuration shared by the initiator modules.
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <zephyr/bluetooth/conn.h>

#define CS_CONFIG_ID 0

#if defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2_SUB_MODE_1)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_1)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_1_NO_SUB_MODE
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_3)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_3_NO_SUB_MODE
#else
BUILD_ASSERT(false, "Invalid ranging mode");
#endif

#define NUM_MODE_0_STEPS 3

/* Maximum number of antenna paths. */
#define MAX_AP (CONFIG_BT_RAS_MAX_ANTENNA_PATHS)

/* CS channel 2 is the first usable channel (index 0 in cs_de arrays). */
#define CHANNEL_INDEX_OFFSET (2)

/* ACL connection interval, in 1.25 ms units. */
#define CONN_INTERVAL 0x10

#endif /* APP_CONFIG_H_ */
