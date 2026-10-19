#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "connection/tracker_roster.h"
#include "system/system.h"

#define RECORD_BYTES 98
struct entry {
	int length;
	uint8_t bytes[RECORD_BYTES + 1];
};
static struct entry storage[TRACKER_ROSTER + 1];
static int read_error_key, read_error, write_error, writes, reads_legacy;
static bool commit_before_error;

int sys_read_checked(uint16_t id, void *data, size_t len)
{
	assert(id <= TRACKER_ROSTER);
	if (id != TRACKER_ROSTER) {
		reads_legacy++;
	}
	if (id == read_error_key) {
		return read_error;
	}
	struct entry *entry = &storage[id];
	if (entry->length < 0) {
		return entry->length;
	}
	size_t copied = (size_t)entry->length < len ? (size_t)entry->length : len;
	memcpy(data, entry->bytes, copied);
	return entry->length;
}

int sys_write_checked(uint16_t id, const void *data, size_t len)
{
	/* No queue exists: every mutation is one synchronous complete snapshot. */
	assert(id == TRACKER_ROSTER && len == RECORD_BYTES);
	writes++;
	if (!write_error || commit_before_error) {
		memcpy(storage[id].bytes, data, len);
		storage[id].length = (int)len;
	}
	return write_error;
}

static void reset(void)
{
	memset(storage, 0, sizeof(storage));
	for (size_t i = 0; i < sizeof(storage) / sizeof(storage[0]); i++) {
		storage[i].length = -ENOENT;
	}
	read_error_key = -1;
	read_error = write_error = writes = reads_legacy = 0;
	commit_before_error = false;
}

static void legacy(uint8_t count, const uint64_t *addresses)
{
	storage[STORED_TRACKERS].length = 1;
	storage[STORED_TRACKERS].bytes[0] = count;
	for (uint8_t i = 0; i < count && i < MAX_TRACKERS; i++) {
		storage[STORED_ADDR_0 + i].length = sizeof(uint64_t);
		memcpy(storage[STORED_ADDR_0 + i].bytes, &addresses[i], sizeof(uint64_t));
	}
}

static void expect_error(int error)
{
	uint8_t count = 91;
	uint64_t output[MAX_TRACKERS], before[MAX_TRACKERS];
	memset(output, 0xa5, sizeof(output));
	memcpy(before, output, sizeof(before));
	int old_writes = writes;
	assert(tracker_roster_load(&count, output) == error);
	assert(count == 91 && memcmp(output, before, sizeof(output)) == 0);
	assert(writes == old_writes);
}

static void expect_roster(uint8_t expected_count, const uint64_t *expected)
{
	uint8_t count = 99;
	uint64_t output[MAX_TRACKERS];
	memset(output, 0xa5, sizeof(output));
	assert(tracker_roster_load(&count, output) == 0);
	assert(count == expected_count);
	for (unsigned i = 0; i < MAX_TRACKERS; i++) {
		assert(output[i] == (i < count ? expected[i] : 0));
	}
}

int main(void)
{
	uint64_t addresses[MAX_TRACKERS];
	for (unsigned i = 0; i < MAX_TRACKERS; i++) {
		addresses[i] = UINT64_C(0x123456789abc) + i;
	}
	assert(!tracker_roster_address_valid(0));
	assert(!tracker_roster_address_valid(UINT64_C(0xffffffffffff)));
	assert(!tracker_roster_address_valid(UINT64_C(1) << 48));
	assert(tracker_roster_address_valid(1));
	assert(tracker_roster_address_valid(UINT64_C(0xfffffffffffe)));

	reset();
	assert(tracker_roster_save(MAX_TRACKERS, addresses) == 0 && writes == 1);
	assert(storage[TRACKER_ROSTER].length == RECORD_BYTES);
	const uint8_t prefix[] = {1, 16, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12};
	assert(memcmp(storage[TRACKER_ROSTER].bytes, prefix, sizeof(prefix)) == 0);
	expect_roster(MAX_TRACKERS, addresses);
	assert(reads_legacy == 0);
	struct entry good = storage[TRACKER_ROSTER];

	const uint8_t bad_counts[] = {17, 255};
	for (unsigned i = 0; i < sizeof(bad_counts); i++) {
		storage[TRACKER_ROSTER] = good;
		storage[TRACKER_ROSTER].bytes[1] = bad_counts[i];
		expect_error(-EINVAL);
		assert(tracker_roster_save(bad_counts[i], addresses) == -EINVAL);
	}
	for (int length = 0; length <= RECORD_BYTES + 1; length++) {
		if (length == RECORD_BYTES) continue;
		storage[TRACKER_ROSTER] = good;
		storage[TRACKER_ROSTER].length = length;
		expect_error(-EINVAL);
	}
	storage[TRACKER_ROSTER] = good;
	storage[TRACKER_ROSTER].bytes[0] = 2;
	expect_error(-EINVAL);
	for (int fill = 0; fill <= 255; fill += 255) {
		storage[TRACKER_ROSTER] = good;
		memset(storage[TRACKER_ROSTER].bytes + 2, fill, 6);
		expect_error(-EINVAL);
	}
	storage[TRACKER_ROSTER] = good;
	memcpy(storage[TRACKER_ROSTER].bytes + 8, storage[TRACKER_ROSTER].bytes + 2, 6);
	expect_error(-EINVAL);
	storage[TRACKER_ROSTER] = good;
	storage[TRACKER_ROSTER].bytes[1] = 1; /* Nonzero inactive slots. */
	expect_error(-EINVAL);
	assert(reads_legacy == 0);

	const uint64_t bad_addresses[] = {0, UINT64_C(0xffffffffffff), UINT64_C(1) << 48, addresses[1]};
	for (unsigned i = 0; i < sizeof(bad_addresses) / sizeof(bad_addresses[0]); i++) {
		reset();
		uint64_t invalid[MAX_TRACKERS];
		memcpy(invalid, addresses, sizeof(invalid));
		invalid[0] = bad_addresses[i];
		assert(tracker_roster_save(2, invalid) == -EINVAL && writes == 0);
		legacy(2, invalid);
		expect_error(-EINVAL);
	}
	for (unsigned i = 0; i < sizeof(bad_counts); i++) {
		reset(); legacy(bad_counts[i], addresses); expect_error(-EINVAL);
	}
	reset(); legacy(2, addresses);
	storage[STORED_TRACKERS].length = 2;
	expect_error(-EINVAL);
	storage[STORED_TRACKERS].length = 1;
	storage[STORED_ADDR_0 + 1].length = -ENOENT;
	expect_error(-EINVAL);
	storage[STORED_ADDR_0 + 1].length = 7;
	expect_error(-EINVAL);
	storage[STORED_ADDR_0 + 1].length = 9;
	expect_error(-EINVAL);

	/* No fallback for I/O errors, even if legacy keys are valid. */
	const int keys[] = {TRACKER_ROSTER, STORED_TRACKERS, STORED_ADDR_0};
	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		reset(); legacy(2, addresses);
		read_error_key = keys[i]; read_error = -EIO;
		expect_error(-EIO);
		if (keys[i] == TRACKER_ROSTER) assert(reads_legacy == 0);
		read_error_key = -1;
		expect_roster(2, addresses);
		assert(writes == 1);
	}

	/* Migration retains old keys, and publishes nothing until commit. */
	reset(); legacy(2, addresses);
	struct entry old_count = storage[STORED_TRACKERS];
	struct entry old_address = storage[STORED_ADDR_0];
	write_error = -ENOSPC;
	uint8_t count = 77;
	uint64_t output[MAX_TRACKERS] = {88};
	assert(tracker_roster_load(&count, output) == -ENOSPC);
	assert(count == 77 && output[0] == 88 && writes == 1);
	assert(storage[TRACKER_ROSTER].length == -ENOENT);
	write_error = 0;
	expect_roster(2, addresses);
	assert(writes == 2);
	assert(memcmp(&old_count, &storage[STORED_TRACKERS], sizeof(old_count)) == 0);
	assert(memcmp(&old_address, &storage[STORED_ADDR_0], sizeof(old_address)) == 0);
	storage[STORED_TRACKERS].bytes[0] = 255;
	expect_roster(2, addresses); /* New record always wins. */
	assert(writes == 2);
	reset();
	expect_roster(0, addresses); /* Missing legacy count migrates empty. */
	assert(writes == 1);

	/* Model NVS atomic replacement across cuts before/after commit. This
	 * tests roster transaction boundaries, not physical flash guarantees. */
	for (int after_commit = 0; after_commit <= 1; after_commit++) {
		uint64_t replacement[MAX_TRACKERS] = {addresses[1]};
		reset();
		assert(tracker_roster_save(2, addresses) == 0);
		write_error = -EIO;
		commit_before_error = after_commit;
		assert(tracker_roster_save(1, replacement) == -EIO);
		assert(writes == 2);
		write_error = 0;
		expect_roster(after_commit ? 1 : 2, after_commit ? replacement : addresses);
		assert(writes == 2);
		reset(); legacy(2, addresses);
		write_error = -EIO; commit_before_error = after_commit;
		count = 77; output[0] = 88;
		assert(tracker_roster_load(&count, output) == -EIO);
		assert(count == 77 && output[0] == 88);
		write_error = 0;
		expect_roster(2, addresses);
		assert(writes == (after_commit ? 1 : 2));
	}
	puts("tracker roster regressions passed");
	return 0;
}
