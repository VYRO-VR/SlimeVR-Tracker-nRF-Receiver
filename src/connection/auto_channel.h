#ifndef AUTO_CHANNEL_H
#define AUTO_CHANNEL_H
#include <stdbool.h>
#include <stdint.h>
/* Beacon family is reserved before tracker dispatch, including malformed frames. */
bool auto_channel_is_beacon(const uint8_t *data, uint8_t length);
/* Nonblocking event-IRQ handoff; true means private frame consumed/rejected. */
bool auto_channel_receive(const uint8_t *data, uint8_t length, uint8_t pipe, uint8_t home);
void auto_channel_init(const uint8_t identity[6]);
void auto_channel_set_enabled(bool enabled);
const char *auto_channel_status(void);
uint32_t auto_channel_generation(void);
/* Caller serializes validation and transaction publication with irq_lock(). */
bool auto_channel_decision_valid(uint32_t expected_generation);
/* Actual tagged transaction result; nonblocking mailbox, never policy mutation. */
void auto_channel_migration_complete(uint32_t generation, uint8_t old_home,
				    uint8_t new_home, bool success);

#define AUTO_CHANNEL_STATUS_PEERS 16
struct auto_channel_peer_status {
	uint8_t identity[6], home, observations;
	uint16_t sequence;
	int64_t last_seen_ms;
	bool fresh;
};
struct auto_channel_energy_status {
	int peak, quiet_floor; /* Positive RSSI magnitudes; -1 means invalid. */
	uint32_t coverage_us, samples;
	uint16_t visits, attempts;
};
struct auto_channel_decision_status {
	bool valid;
	int64_t time_ms;
	uint32_t generation;
	uint8_t home, target;
	struct auto_channel_energy_status reference, candidate;
	uint64_t excluded; /* Bit n represents even channel 2*n, through 88. */
	const char *reason; /* Immutable static string, never mutable policy storage. */
};
struct auto_channel_snapshot {
	bool initialized, enabled, interference_latched, radio_fault, entropy_fault;
	bool pairing, table_full;
	uint8_t identity[6], home, peer_count;
	uint32_t generation, tx_attempts, tx_completed;
	uint32_t required_mask, ready_mask, recovered_mask;
	uint8_t layout_epoch;
	bool cap_pending;
	int64_t now_ms;
	const char *phase; /* Immutable static string. */
	uint64_t excluded;
	struct auto_channel_peer_status peers[AUTO_CHANNEL_STATUS_PEERS];
	struct auto_channel_decision_status decision;
};
/* One bounded coherent copy. Formatting never holds the publication lock. */
void auto_channel_get_snapshot(struct auto_channel_snapshot *out);
#endif
