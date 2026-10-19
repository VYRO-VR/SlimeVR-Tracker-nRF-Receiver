#include "auto_channel_policy.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void)
{
	struct ac_policy high = {.identity = {2}}, low = {.identity = {1}};
	uint8_t f[16];
	int16_t scores[101];
	for (unsigned ch=0; ch<101; ch++) scores[ch]=-1;
	ac_frame(f, low.identity, 0, 1);
	assert(ac_observe(&high, f, 0));
	assert(!ac_observe(&high, f, 1));
	for (int i = 2; i <= 3; i++) { ac_frame(f, low.identity, 0, i); assert(ac_observe(&high, f, i)); }
	assert(ac_occupied(&high, 0, 4));
	ac_frame(f, low.identity, 0, 2); assert(!ac_observe(&high, f, 4));
	for (int i = 1; i <= 3; i++) { ac_frame(f, high.identity, 0, i); assert(ac_observe(&low, f, i)); }
	assert(ac_occupied(&low, 0, 4));
	scores[0] = 50; scores[2] = 49;
	assert(ac_choose(&high, 0, scores, 4) == -1);
	/* Energy ranking never vetoes a lossy lower-ID receiver. */
	scores[2] = 50;
	assert(ac_choose(&high, 0, scores, 4) == -1);
	scores[2] = 51;
	assert(ac_choose(&high, 0, scores, 4) == 2);
	assert(ac_choose(&low, 0, scores, 4) == 2);
	scores[1] = 100;
	assert(ac_choose(&high, 0, scores, 4) == 2);
	scores[1] = 0; scores[100] = 90;
	assert(ac_choose(&high, 0, scores, 4) == 2);
	/* A stronger score cannot promote odd or out-of-range destinations. */
	for (unsigned ch = 3; ch <= 100; ch++) {
		scores[ch] = 100;
		int selected = ac_choose(&high, 0, scores, 4);
		assert(selected == ((ch <= 88 && !(ch & 1)) ? (int)ch : 2));
		scores[ch] = 0;
	}
	/* 88 is the highest automatic destination. */
	scores[88] = 90;
	assert(ac_choose(&high, 0, scores, 4) == 88);
	scores[88] = 0;
	/* 90 is even but explicitly rejected as a destination. */
	scores[90] = 90;
	assert(ac_choose(&high, 0, scores, 4) == 2);
	scores[90] = 0;
	uint8_t third[6] = {3}; ac_frame(f, third, 100, 1);
	assert(ac_observe(&high, f, 4));
	assert(ac_occupied(&high, 99, 4));
	scores[99] = 100;
	assert(ac_choose(&high, 0, scores, 4) == 2);
	/* Cooldown belongs to loss admission, never energy ranking. */
	high.cooldown = 100; assert(ac_choose(&high, 0, scores, 99) == 2);
	assert(ac_choose(&high, 0, scores, 100) == 2);
	assert(!ac_occupied(&high, 100, AC_EXPIRY_MS + 4));
	ac_frame(f, high.identity, 0, 8); assert(!ac_observe(&high, f, 8));
	ac_frame(f, third, 101, 8); assert(!ac_observe(&high, f, 8));
	ac_frame(f, third, 2, 8); f[3] = 2; assert(!ac_observe(&high, f, 8));
	ac_frame(f, third, 2, 8); f[15] = 1; assert(!ac_observe(&high, f, 8));
	uint8_t zero[6] = {0}; ac_frame(f, zero, 2, 8); assert(!ac_observe(&high, f, 8));
	/* A manually selected high/odd home is still a valid conflict source;
	 * channel zero remains a valid automatic destination. */
	struct ac_policy outside = {.identity = {2}};
	for (int i = 1; i <= 3; i++) {
		ac_frame(f, low.identity, 99, i);
		assert(ac_observe(&outside, f, i));
	}
	memset(scores, 0, sizeof(scores));
	scores[99] = 50; scores[0] = 60;
	assert(ac_choose(&outside, 99, scores, 4) == 0);
	/* A manual 90 home is still a measured reference and conflict source even
	 * though it is outside the automatic destination range: its own score
	 * anchors the comparison while 90 is never a candidate. */
	struct ac_policy pinned = {.identity = {2}};
	for (int i = 1; i <= 3; i++) {
		ac_frame(f, low.identity, 90, i);
		assert(ac_observe(&pinned, f, i));
	}
	memset(scores, 0, sizeof(scores));
	scores[90] = 50; scores[0] = 60;
	assert(ac_choose(&pinned, 90, scores, 4) == 0);
	scores[0] = 0; scores[88] = 100;
	assert(ac_choose(&pinned, 90, scores, 4) == 88);
	struct ac_policy alone = {.identity = {7}};
	memset(scores, 0, sizeof(scores));
	scores[2] = 50; scores[0] = 50;
	assert(ac_choose(&alone, 2, scores, 0) == -1); /* equal quiet HOME stays */
	scores[0] = 51;
	ac_frame(f, third, 1, 9); assert(ac_observe(&alone, f, 9));
	assert(ac_choose(&alone, 2, scores, 9) == -1); /* known adjacent peer */
	memset(&alone, 0, sizeof(alone));
	for (unsigned ch=0; ch<101; ch++) scores[ch]=-1;
	scores[18]=0;
	assert(ac_choose(&alone,18,scores,0)==-1);
	scores[26]=0;
	assert(ac_choose(&alone,18,scores,0)==-1); /* valid physical zero ties */
	scores[26]=1;
	assert(ac_choose(&alone,18,scores,0)==26);
	scores[18]=-1;
	assert(ac_choose(&alone,18,scores,0)==-1); /* invalid home */
	scores[18]=50; scores[26]=96; scores[88]=95;
	assert(ac_choose(&alone,18,scores,0)==26);
	scores[24]=96; assert(ac_choose(&alone,18,scores,0)==24);
	ac_frame(f, third, 25, 1); assert(ac_observe(&alone,f,0));
	assert(ac_choose(&alone,18,scores,0)==88);
	puts("auto channel peer validation/expiry/energy ranking/known-neighbor exclusion PASS");
}
