/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLPR_SHARED_H_
#define FLPR_SHARED_H_

#define FLPR_SHARED_MEMORY_START 0x2fc12fe0

struct flpr_shared {
	uint8_t iso_out_ep_enabled;
	uint8_t iso_in_ep_enabled;
	uint8_t flpr_uses_iso_out;
	uint8_t flpr_uses_iso_in;
} __packed;

/* Ideally this should be replaced with the struct placed at fixed memory
 * location, but I have no idea how to get it working so both App core and FLPR
 * would get the same location.
 *
 * The approach with just hardcoding addresses is really really bad. However it
 * is a working solution to the problem.
 */
#define FLPR_SHARED(member) ((volatile struct flpr_shared *)FLPR_SHARED_MEMORY_START)->member

#endif /* FLPR_SHARED_H_ */
