#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>
#include <sched.h>
#define K_FOREVER 0
#define RCV_HID_CMD_LEN 16
#define RCV_HID_TYPE_CMD_ACK 251
#define RCV_HID_ST_OK 0
#define RCV_HID_ST_STARTED 1
#define RCV_HID_ST_ENOENT 2
#define RCV_HID_ST_EBUSY 3
#define RCV_HID_OP_TRACKER_CH_ALL 100
#define RCV_HID_OP_TRACKER_CH_CLR 101
typedef atomic_int atomic_t;
static void atomic_set(atomic_t *a, int value) { atomic_store(a, value); }
static bool atomic_cas(atomic_t *a, int expected, int value) {
    return atomic_compare_exchange_strong(a, &expected, value);
}
static pthread_mutex_t channel_completion_lock = PTHREAD_MUTEX_INITIALIZER;
static atomic_t ch_hid_pending;
static uint8_t ch_hid_seq, ch_hid_opcode;
static void (*async_ack_fn)(const uint8_t *);
static atomic_bool callback_waiting;
static _Thread_local bool callback_thread;
static void k_mutex_lock(pthread_mutex_t *lock, int timeout) {
    if (callback_thread) atomic_store(&callback_waiting, true);
    assert(pthread_mutex_lock(lock) == 0);
}
static void k_mutex_unlock(pthread_mutex_t *lock) { assert(pthread_mutex_unlock(lock) == 0); }
static void on_tracker_channel_done(bool success);
static pthread_t completion_thread;
static bool reject, completion_success;
static unsigned ack_count;
static uint8_t last_ack[RCV_HID_CMD_LEN];
static void save_ack(const uint8_t *ack) { memcpy(last_ack, ack, sizeof(last_ack)); ack_count++; }
static void *complete(void *arg) {
    callback_thread = true;
    on_tracker_channel_done(completion_success);
    return NULL;
}
static uint8_t start(void) {
    if (reject) return RCV_HID_ST_EBUSY;
    atomic_store(&callback_waiting, false);
    assert(pthread_create(&completion_thread, NULL, complete, NULL) == 0);
    /* The callback MUST already be trying to run before STARTED returns.
     * Without the production start+arm mutex, it observes no registration. */
    while (!atomic_load(&callback_waiting)) sched_yield();
    return RCV_HID_ST_STARTED;
}
static uint8_t rcv_cmd_tracker_channel_all(uint8_t channel) { return start(); }
static uint8_t rcv_cmd_tracker_channel_clear_all(void) { return start(); }
/* PRODUCTION BODIES */
int main(void) {
    async_ack_fn = save_ack;
    for (unsigned opcode = 100; opcode <= 101; opcode++) {
        for (unsigned success = 0; success <= 1; success++) {
            completion_success = success;
            unsigned before = ack_count;
            uint8_t seq = (uint8_t)(opcode + success);
            assert(hid_start(seq, opcode) == RCV_HID_ST_STARTED);
            assert(pthread_join(completion_thread, NULL) == 0);
            assert(ack_count == before + 1);
            assert(last_ack[0] == RCV_HID_TYPE_CMD_ACK && last_ack[1] == seq && last_ack[2] == opcode);
            assert(last_ack[3] == (success ? RCV_HID_ST_OK : RCV_HID_ST_ENOENT));
            assert(atomic_load(&ch_hid_pending) == 0);
        }
    }
    /* A rejected overlapping request must not replace the accepted sequence. */
    arm_ch_hid_completion(9, 100);
    reject = true;
    assert(hid_start(10, 101) == RCV_HID_ST_EBUSY);
    on_tracker_channel_done(true);
    assert(last_ack[1] == 9 && last_ack[2] == 100);
    unsigned before = ack_count;
    on_tracker_channel_done(false);
    assert(ack_count == before);
    puts("PASS channel HID completion: callback-before-return, SET/CLEAR success/failure, overlapping request isolation");
    return 0;
}
