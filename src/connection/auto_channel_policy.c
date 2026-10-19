#include "auto_channel_policy.h"
#include <string.h>

/* Private protocol, deliberately not the upstream receiver broadcast format.
 * RADIO CRC protects the frame; magic/version/reserved fields reject unrelated RF. */
void ac_frame(uint8_t out[AC_FRAME_SIZE], const uint8_t id[6], uint8_t channel, uint16_t sequence)
{
	memset(out, 0, AC_FRAME_SIZE);
	memcpy(out, "SAC\x01", 4);
	memcpy(out + 4, id, 6);
	out[10] = channel;
	out[11] = sequence;
	out[12] = sequence >> 8;
}

static bool fresh(const struct ac_peer *peer, int64_t now)
{
	return peer->observations && now >= peer->seen && now - peer->seen < AC_EXPIRY_MS;
}

bool ac_observe(struct ac_policy *p, const uint8_t frame[AC_FRAME_SIZE], int64_t now)
{
	static const uint8_t zero[6];
	if (memcmp(frame, "SAC\x01", 4) || frame[10] > 100 || frame[13] || frame[14] || frame[15] ||
	    !memcmp(frame + 4, zero, 6) || !memcmp(frame + 4, p->identity, 6)) return false;
	struct ac_peer *slot = NULL;
	for (unsigned i = 0; i < AC_PEERS; i++) {
		if (!memcmp(p->peers[i].id, frame + 4, 6)) { slot = &p->peers[i]; break; }
		if (!fresh(&p->peers[i], now) && !slot) slot = &p->peers[i];
	}
	/* A full peer table is not evidence that an occupied channel is free. */
	if (!slot) return false;
	uint16_t sequence = frame[11] | ((uint16_t)frame[12] << 8);
	bool known = fresh(slot, now) && !memcmp(slot->id, frame + 4, 6);
	if (known && (int16_t)(sequence - slot->sequence) <= 0) return false;
	bool same = known && slot->channel == frame[10];
	uint8_t count = same ? slot->observations : 0;
	memcpy(slot->id, frame + 4, 6);
	slot->channel = frame[10];
	slot->sequence = sequence;
	slot->observations = count < 3 ? count + 1 : 3;
	slot->seen = now;
	return true;
}

bool ac_occupied(const struct ac_policy *p, uint8_t channel, int64_t now)
{
	for (unsigned i = 0; i < AC_PEERS; i++) {
		int distance = (int)p->peers[i].channel - channel;
		/* 2Mbps occupies adjacent MHz channels, including explicit odd ones. */
		if (fresh(&p->peers[i], now) && distance >= -1 && distance <= 1) return true;
	}
	return false;
}


int ac_choose(const struct ac_policy *p, uint8_t home, const int16_t score[101], int64_t now)
{
	if (home > 100 || score[home] < 0) return -1;
	int best = -1;
	/* Saturation: defer rather than assume unrecorded peers don't exist. */
	unsigned live = 0;
	for (unsigned i = 0; i < AC_PEERS; i++) live += fresh(&p->peers[i], now);
	if (live == AC_PEERS) return -1;
	/* Automatic destinations: even 0..88 only (no odd channel, no 90). */
	for (unsigned ch = 0; ch <= 88; ch += 2) {
		/* Compare matched, guarded visits after paired-address ambiguity was
		 * discarded on BOTH home and candidates. Equality stays HOME.
		 * Unrecognized traffic and unseen foreign peers remain unknown. */
		if (ch == home || score[ch] < 0 || ac_occupied(p, ch, now) || score[ch] <= score[home]) continue;
		if (best < 0 || score[ch] > score[best]) best = ch;
	}
	return best;
}
