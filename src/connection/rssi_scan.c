/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2026 SlimeVR Contributors
 */

#include "connection/rssi_scan.h"

#include "connection/esb.h"

#include <esb.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <stdbool.h>
#include <stdint.h>

#include <hal/nrf_radio.h>

#define RSSI_SCAN_FIRST_CHANNEL 0
#define RSSI_SCAN_LAST_CHANNEL 88

#define RSSI_SCAN_SAMPLES_PER_CH 50
#define RSSI_SCAN_SWEEPS 200

// Optional spacing. 0 = as fast as possible.
#define RSSI_SCAN_INTER_SAMPLE_US 0
#define RSSI_SCAN_INTER_SWEEP_MS 1

// RADIO->RSSISAMPLE is 7-bit, 0..127, representing -dBm.
// Higher sample => weaker energy (cleaner channel).
#define RSSI_SAMPLE_MAX 127

#define RSSI_SCAN_READY_TIMEOUT_US 200
#define RSSI_SCAN_RSSIEND_TIMEOUT_US 200
#define RSSI_SCAN_DISABLED_TIMEOUT_US 200

static bool wait_for_event_and_clear(volatile uint32_t *evt_reg, uint32_t timeout_us)
{
	if (timeout_us == 0) {
		while (!*evt_reg) {
			/* busy wait */
		}
		*evt_reg = 0;
		return true;
	}

	// Busy-wait with 1us granularity.
	for (uint32_t i = 0; i < timeout_us; i++) {
		if (*evt_reg) {
			*evt_reg = 0;
			return true;
		}
		k_busy_wait(1);
	}
	return false;
}

/**
 * Measure a channel by repeatedly triggering RSSI measurement and return the
 * minimum observed sample.
 *
 * Note:
 * - RSSISAMPLE represents -dBm.
 * - Therefore MIN(sample) corresponds to the strongest energy observed (lowest case).
 */
static uint8_t rssi_scan_channel_measure_min_n(uint8_t channel, uint32_t samples)
{
	// The RADIO->FREQUENCY register expects an offset in MHz from 2400 MHz.
	NRF_RADIO->FREQUENCY = channel;

	// Clear events we depend on.
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_RSSIEND = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_RADIO->TASKS_RXEN = 1;
	if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_READY, RSSI_SCAN_READY_TIMEOUT_US)) {
		NRF_RADIO->TASKS_DISABLE = 1;
		(void)wait_for_event_and_clear(&NRF_RADIO->EVENTS_DISABLED, RSSI_SCAN_DISABLED_TIMEOUT_US);
		return 0;
	}

	uint8_t min_sample = RSSI_SAMPLE_MAX;
	for (uint32_t i = 0; i < samples; i++) {
		NRF_RADIO->EVENTS_RSSIEND = 0;
		NRF_RADIO->TASKS_RSSISTART = 1;
		if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_RSSIEND, RSSI_SCAN_RSSIEND_TIMEOUT_US)) {
			min_sample = 0;
			break;
		}

		uint8_t sample = (uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7F);
		if (sample < min_sample) {
			min_sample = sample;
		}

		if (RSSI_SCAN_INTER_SAMPLE_US) {
			k_busy_wait(RSSI_SCAN_INTER_SAMPLE_US);
		}
	}

	NRF_RADIO->TASKS_DISABLE = 1;
	if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_DISABLED, RSSI_SCAN_DISABLED_TIMEOUT_US)) {
		// If disable doesn't complete, return a conservative value.
		return 0;
	}

	return min_sample;
}

static uint8_t get_current_effective_channel(void)
{
	uint8_t ch = esb_get_receiver_channel();
	if (ch == 0xFF || ch < RSSI_SCAN_FIRST_CHANNEL || ch > RSSI_SCAN_LAST_CHANNEL) {
		ch = CONFIG_RADIO_RF_CHANNEL;
	}
	return ch;
}

static bool better_channel(
	uint8_t ch_a,
	uint8_t lowest_a,
	uint8_t ch_b,
	uint8_t lowest_b,
	bool ch_b_valid,
	uint8_t current_ch
)
{
	if (!ch_b_valid) {
		return true;
	}
	// Primary: maximize lowest (higher = cleaner).
	if (lowest_a > lowest_b) {
		return true;
	}
	if (lowest_a < lowest_b) {
		return false;
	}

	// Tie-breaker: stay close to current channel to reduce disruption.
	int dist_a = (ch_a > current_ch) ? (ch_a - current_ch) : (current_ch - ch_a);
	int dist_b = (ch_b > current_ch) ? (ch_b - current_ch) : (current_ch - ch_b);
	if (dist_a < dist_b) {
		return true;
	}
	if (dist_a > dist_b) {
		return false;
	}

	// Final tie-breaker: smaller channel number.
	return ch_a < ch_b;
}

void rssi_scan_run_and_print(void)
{
	// Ensure HF clock is running (best-effort). This is already started by ESB thread,
	// but keeping it explicit makes the scan less fragile.
	(void)clocks_start();

	printk(
		"RSSI scan (ESB channels %u..%u): samples/ch/sweep=%u, sweeps=%d (total=%u samples/ch)\n",
		RSSI_SCAN_FIRST_CHANNEL,
		RSSI_SCAN_LAST_CHANNEL,
		(uint32_t)RSSI_SCAN_SAMPLES_PER_CH,
		RSSI_SCAN_SWEEPS,
		(uint32_t)RSSI_SCAN_SAMPLES_PER_CH * (uint32_t)RSSI_SCAN_SWEEPS
	);

	// Pause ESB RX (manual diagnostic; link will be interrupted).
	(void)esb_stop_rx();
	esb_deinitialize();
	k_msleep(2);

	// sweep_min[sweep][ch] = per-sweep minimum RSSI sample for that channel.
	static uint8_t sweep_min[RSSI_SCAN_SWEEPS][RSSI_SCAN_LAST_CHANNEL + 1];
	static uint8_t score_lowest[RSSI_SCAN_LAST_CHANNEL + 1];
	static bool measured[RSSI_SCAN_LAST_CHANNEL + 1];

	for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
		for (int sweep = 0; sweep < RSSI_SCAN_SWEEPS; sweep++) {
			sweep_min[sweep][ch] = RSSI_SAMPLE_MAX;
		}
		measured[ch] = false;
	}

	for (int sweep = 0; sweep < RSSI_SCAN_SWEEPS; sweep++) {
		// printk("RSSI scan sweep %d/%d...\n", sweep + 1, RSSI_SCAN_SWEEPS);
		for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
			if (!esb_channel_is_allowed(ch)) {
				continue; // only preferred channels
			}
			measured[ch] = true;
			sweep_min[sweep][ch] = rssi_scan_channel_measure_min_n(ch, RSSI_SCAN_SAMPLES_PER_CH);
		}

		// Yield to other threads/USB log processing.
		if (RSSI_SCAN_INTER_SWEEP_MS) {
			k_msleep(RSSI_SCAN_INTER_SWEEP_MS);
		} else {
			k_yield();
		}
	}

	// Reduce sweeps into per-channel scores (only lowest).
	for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
		if (!measured[ch]) {
			continue;
		}
		uint8_t lowest = RSSI_SAMPLE_MAX;

		for (int sweep = 0; sweep < RSSI_SCAN_SWEEPS; sweep++) {
			uint8_t v = sweep_min[sweep][ch];
			if (v < lowest) {
				lowest = v;
			}
		}

		score_lowest[ch] = lowest;
	}

	uint8_t current_ch = get_current_effective_channel();

	/* Candidate list: measured channels only (score != 0xFF sentinel). */
	enum { NUM_CH = RSSI_SCAN_LAST_CHANNEL - RSSI_SCAN_FIRST_CHANNEL + 1 };
	uint8_t cand_ch[NUM_CH];
	int cand_count = 0;
	for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
		if (measured[ch]) {
			cand_ch[cand_count++] = ch;
		}
	}

	uint8_t recommended = 0;
	bool has_recommended = false;
	for (int i = 0; i < cand_count; i++) {
		uint8_t ch = cand_ch[i];
		if (better_channel(
				ch,
				score_lowest[ch],
				recommended,
				has_recommended ? score_lowest[recommended] : 0,
				has_recommended,
				current_ch
			)) {
			recommended = ch;
			has_recommended = true;
		}
	}

	printk("Current channel (effective): %u\n", current_ch);
	printk(
		"Recommended: channel %u (-%d dBm)\n\n",
		recommended,
		score_lowest[recommended]
	);

	// Sort measured channels only, using simple insertion sort.
	uint8_t sorted_ch[NUM_CH];
	for (int i = 0; i < cand_count; i++) {
		sorted_ch[i] = cand_ch[i];
	}
	for (int i = 1; i < cand_count; i++) {
		uint8_t key = sorted_ch[i];
		int j = i - 1;
		while (j >= 0
			   && better_channel(
				   key,
				   score_lowest[key],
				   sorted_ch[j],
				   score_lowest[sorted_ch[j]],
				   true,
				   current_ch
			   )) {
			sorted_ch[j + 1] = sorted_ch[j];
			j--;
		}
		sorted_ch[j + 1] = key;
	}

	// Print all channels in compact table format
	// Format: "CH(WST)" where WST is lowest (min across all samples).
	printk("All channels sorted by RSSI (lower is better):\n");
	enum { PER_LINE = 8 };
	for (int i = 0; i < cand_count; i++) {
		uint8_t ch = sorted_ch[i];
		printk("%3u (-%2d dBm)", ch, score_lowest[ch]);
		if ((i + 1) % PER_LINE == 0 || i == cand_count - 1) {
			printk("\n");
			k_msleep(20);
		} else {
			printk("  ");
		}
	}

	// Resume ESB RX using unified addresses (current receiver state).
	esb_set_addr_unified();
	(void)esb_initialize(false);
	(void)esb_start_rx();
}
