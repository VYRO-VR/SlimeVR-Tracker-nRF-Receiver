#ifndef SLIMENRF_TRACKER_ROSTER_H
#define SLIMENRF_TRACKER_ROSTER_H

#include <stdbool.h>
#include <stdint.h>
#include "globals.h"

/* Thread-context storage only. Callers serialize mutations and publish only
 * after success. On load failure both output arguments remain untouched. */
int tracker_roster_load(uint8_t *count, uint64_t addresses[MAX_TRACKERS]);
int tracker_roster_save(uint8_t count, const uint64_t addresses[MAX_TRACKERS]);
bool tracker_roster_address_valid(uint64_t address);

#endif
