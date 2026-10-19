/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2026 SlimeVR Contributors
 */
#pragma once
#include <stdint.h>

/* Equal timed slices; magnitudes are positive -dBm. No packet identity is
 * inferred. Failed/truncated slices leave output unchanged. Caller owns radio,
 * enters Disabled and restores HOME. Deadlines are soft, not hard IRQ bounds. */
#define RSSI_SCAN_SLICE_US 500
#define RSSI_SCAN_SLOT_US 100
struct rssi_scan_observation {
	uint8_t peak, quiet_floor;
	uint16_t samples;
	uint32_t coverage_us;
};
int rssi_scan_measure_bounded(uint8_t channel, struct rssi_scan_observation *out);

/**
 * Scan the preferred subset of channels 0..88 and print results/recommendation.
 * Any failed visit invalidates that channel for the entire manual scan.
 *
 * Notes:
 * - This is a blocking call intended for manual console use.
 * - The scan will temporarily stop ESB RX and reinitialize ESB RX when done.
 */
void rssi_scan_run_and_print(void);
