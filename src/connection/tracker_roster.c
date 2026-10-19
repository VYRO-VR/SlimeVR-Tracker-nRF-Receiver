#include "tracker_roster.h"
#include "system/system.h"

#include <errno.h>
#include <string.h>

/* On-flash layout: version, count, sixteen little-endian 48-bit addresses.
 * Inactive slots are zero; no native struct padding or integer byte order. */
#define ROSTER_VERSION 1
#define ROSTER_SLOTS 16
#define ROSTER_ADDRESS_BYTES 6
#define ROSTER_RECORD_BYTES (2 + ROSTER_SLOTS * ROSTER_ADDRESS_BYTES)

_Static_assert(MAX_TRACKERS == ROSTER_SLOTS, "Roster format requires 16 slots");

bool tracker_roster_address_valid(uint64_t address)
{
	return address != 0 && address < UINT64_C(0xffffffffffff);
}

static bool roster_valid(uint8_t count, const uint64_t addresses[MAX_TRACKERS])
{
	if (count > MAX_TRACKERS) {
		return false;
	}
	for (uint8_t i = 0; i < count; i++) {
		if (!tracker_roster_address_valid(addresses[i])) {
			return false;
		}
		for (uint8_t j = 0; j < i; j++) {
			if (addresses[i] == addresses[j]) {
				return false;
			}
		}
	}
	return true;
}

int tracker_roster_save(uint8_t count, const uint64_t addresses[MAX_TRACKERS])
{
	if (!roster_valid(count, addresses)) {
		return -EINVAL;
	}
	uint8_t record[ROSTER_RECORD_BYTES] = {ROSTER_VERSION, count};
	for (uint8_t i = 0; i < count; i++) {
		for (uint8_t j = 0; j < ROSTER_ADDRESS_BYTES; j++) {
			record[2 + i * ROSTER_ADDRESS_BYTES + j] = addresses[i] >> (8 * j);
		}
	}
	return sys_write_checked(TRACKER_ROSTER, record, sizeof(record));
}

int tracker_roster_load(uint8_t *count, uint64_t addresses[MAX_TRACKERS])
{
	uint8_t record[ROSTER_RECORD_BYTES];
	uint64_t loaded[MAX_TRACKERS] = {0};
	uint8_t loaded_count = 0;
	int err = sys_read_checked(TRACKER_ROSTER, record, sizeof(record));
	if (err >= 0) {
		if (err != (int)sizeof(record) || record[0] != ROSTER_VERSION ||
			record[1] > MAX_TRACKERS) {
			return -EINVAL;
		}
		loaded_count = record[1];
		for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
			for (uint8_t j = 0; j < ROSTER_ADDRESS_BYTES; j++) {
				loaded[i] |= (uint64_t)record[2 + i * ROSTER_ADDRESS_BYTES + j] << (8 * j);
			}
			if (i >= loaded_count && loaded[i] != 0) {
				return -EINVAL;
			}
		}
		if (!roster_valid(loaded_count, loaded)) {
			return -EINVAL;
		}
	} else if (err == -ENOENT) {
		err = sys_read_checked(STORED_TRACKERS, &loaded_count, sizeof(loaded_count));
		if (err == -ENOENT) {
			loaded_count = 0;
		} else if (err < 0) {
			return err;
		} else if (err != (int)sizeof(loaded_count) || loaded_count > MAX_TRACKERS) {
			return -EINVAL;
		}
		for (uint8_t i = 0; i < loaded_count; i++) {
			err = sys_read_checked(STORED_ADDR_0 + i, &loaded[i], sizeof(loaded[i]));
			if (err < 0) {
				return err == -ENOENT ? -EINVAL : err;
			}
			if (err != (int)sizeof(loaded[i])) {
				return -EINVAL;
			}
		}
		if (!roster_valid(loaded_count, loaded)) {
			return -EINVAL;
		}
		/* Commit migration before returning any state; old keys stay intact. */
		err = tracker_roster_save(loaded_count, loaded);
		if (err) {
			return err;
		}
	} else {
		return err;
	}
	memcpy(addresses, loaded, sizeof(loaded));
	*count = loaded_count;
	return 0;
}
