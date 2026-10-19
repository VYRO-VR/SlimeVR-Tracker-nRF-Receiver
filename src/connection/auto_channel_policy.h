#ifndef AUTO_CHANNEL_POLICY_H
#define AUTO_CHANNEL_POLICY_H
#include <stdbool.h>
#include <stdint.h>
#define AC_FRAME_SIZE 16
#define AC_PEERS 16
#define AC_EXPIRY_MS 120000
#define AC_COOLDOWN_MS 120000
struct ac_peer { uint8_t id[6], channel, observations; uint16_t sequence; int64_t seen; };
struct ac_policy { uint8_t identity[6]; struct ac_peer peers[AC_PEERS]; int64_t cooldown; };
void ac_frame(uint8_t out[AC_FRAME_SIZE], const uint8_t id[6], uint8_t channel, uint16_t sequence);
bool ac_observe(struct ac_policy *p, const uint8_t frame[AC_FRAME_SIZE], int64_t now);
bool ac_occupied(const struct ac_policy *p, uint8_t channel, int64_t now);
/* Negative scores invalid; scores are strongest energy in guarded visits.
 * Caller guarantees matched temporal coverage and discards paired-address
 * ambiguous visits symmetrically. Strict improvement only; equality stays HOME. */
int ac_choose(const struct ac_policy *p, uint8_t home, const int16_t score[101], int64_t now);
#endif
