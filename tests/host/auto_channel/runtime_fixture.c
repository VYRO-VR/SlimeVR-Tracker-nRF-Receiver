#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include "auto_channel_policy.h"
#include "auto_channel.h"
#include "rssi_scan.h"
#include "rssi_scan_fem.h"
#define CONFIG_RADIO_AUTO_CHANNEL 1
#define CONFIG_RADIO_RF_CHANNEL 2
#define __aligned(x) __attribute__((aligned(x)))
#define LOG_LEVEL_INF 0
#define LOG_MODULE_REGISTER(...)
static unsigned warnings;
static char warning_text[512];
#define LOG_WRN(...) do { warnings++; snprintf(warning_text,sizeof(warning_text),__VA_ARGS__); } while (0)
#define LOG_ERR(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define K_SEM_DEFINE(name,a,b) int name
#define K_THREAD_DEFINE(...)
#define K_FOREVER -1
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define ARG_UNUSED(x) (void)(x)
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
typedef int atomic_t;
static int atomic_get(atomic_t *a) { return *a; }
static void atomic_set(atomic_t *a, int b) { *a = b; }
static void atomic_inc(atomic_t *a) { (*a)++; }
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
/* FIFO copies bytes exactly as Zephyr's fixed-size ISR queue does. */
struct k_msgq { unsigned char *data; size_t size; unsigned capacity, count, first; };
#define K_MSGQ_DEFINE(name,size,count,alignment) \
 unsigned char name##_storage[(size)*(count)]; \
 struct k_msgq name = {name##_storage,(size),(count),0,0}
static int k_msgq_put(struct k_msgq *q, const void *data, int wait) {
 assert(wait==K_NO_WAIT);
 if (q->count==q->capacity) return -ENOMSG;
 memcpy(q->data+((q->first+q->count)%q->capacity)*q->size,data,q->size); q->count++; return 0;
}
static int k_msgq_get(struct k_msgq *q, void *data, int wait) {
 assert(wait==K_NO_WAIT);
 if (!q->count) return -ENOMSG;
 memcpy(data,q->data+q->first*q->size,q->size); q->first=(q->first+1)%q->capacity; q->count--; return 0;
}
static void k_msgq_purge(struct k_msgq *q) { q->count=q->first=0; }
#define RADIO_MODE_MODE_Nrf_2Mbit 1
#define RADIO_TXPOWER_TXPOWER_0dBm 0
#define RADIO_STATE_STATE_Disabled 0
#define RADIO_SHORTS_END_START_Msk 0x20
static struct {
 uint32_t EVENTS_DISABLED,TASKS_DISABLE,EVENTS_END,INTENCLR,MODE,FREQUENCY,TXPOWER,PCNF0,PCNF1,BASE0,PREFIX0,TXADDRESS,RXADDRESSES,CRCCNF,CRCINIT,CRCPOLY,SHORTS,TASKS_TXEN,TASKS_RXEN,CRCSTATUS,STATE;
 uint32_t EVENTS_READY,EVENTS_RSSIEND,TASKS_RSSISTART,TASKS_START,RSSISAMPLE;
 uint32_t EVENTS_ADDRESS;
 uintptr_t PACKETPTR;
} hw;
#define NRF_RADIO (&hw)
static uint64_t tick;
static bool owner, busy, pairing, fail_disable, fail_restore, fail_start, fail_stop, fail_tx;
static unsigned begins, releases, restores, migrations, transmitted, quiesces;
static uint8_t measured_ch[8192], beacon_ch[256];
static int64_t beacon_time[256];
static int64_t measured_time[8192];
static unsigned measured_n;
static unsigned raw_samples[101], raw_ends;
static int raw_fail_channel;
static bool raw_fail_ready, raw_fail_sample, raw_packet_end, raw_zero;
static uint8_t mock_channel, migrated_to;
static uint32_t k_cycle_get_32(void) { return tick; }
static uint32_t k_cyc_to_us_floor32(uint32_t n) { return n; }
static uint32_t k_us_to_cyc_ceil32(uint32_t n) { return n; }
static int64_t k_uptime_get(void) { return tick / 1000; }
struct device { int unused; };
static const struct device rng_device;
static bool fail_entropy;
static unsigned seed_reads;
#define DT_NODELABEL(name) 0
#define DEVICE_DT_GET(node) (&rng_device)
static bool device_is_ready(const struct device *dev) { (void)dev; return true; }
static int entropy_get_entropy(const struct device *dev, uint8_t *out, uint16_t len) {
 (void)dev; assert(!owner); seed_reads++;
 if (fail_entropy) return -EIO;
 assert(len==sizeof(uint32_t)); uint32_t seed=4321; memcpy(out,&seed,len); return 0;
}
/* Independent physical-time waveform: a 1ms burst every 10ms. Tasks complete
 * after READY40/RSSI8/DISABLED2 us, not synchronously when software asks.
 * Scene1 is a bad candidate hidden by millisecond-phase aliasing; scene2
 * puts the burst on HOME over an otherwise quiet floor. */
static unsigned energy_scene;
static uint64_t energy_ready_at, energy_sample_at, energy_disabled_at;
static unsigned periodic_energy(uint8_t channel) {
 bool burst=tick%10000<1000;
 if(energy_scene==2) {
  if(channel==88) return burst?45:96;
  return channel==74?96:40;
 }
 if(channel==88) return 80;
 if(channel==52) return burst?45:96;
 return 70;
}
static void periodic_radio_wait(unsigned us) {
 if(hw.TASKS_RXEN) {
  hw.TASKS_RXEN=0; hw.STATE=2; energy_ready_at=tick+40;
  assert(measured_n<sizeof(measured_ch));
  measured_time[measured_n]=k_uptime_get(); measured_ch[measured_n++]=hw.FREQUENCY;
 }
 if(hw.TASKS_START) { hw.TASKS_START=0; hw.STATE=1; }
 if(hw.TASKS_RSSISTART) { hw.TASKS_RSSISTART=0; energy_sample_at=tick+8; }
 if(hw.TASKS_DISABLE) { hw.TASKS_DISABLE=0; energy_disabled_at=tick+2; }
 tick+=us;
 if(energy_ready_at && tick>=energy_ready_at) { hw.EVENTS_READY=1; energy_ready_at=0; }
 if(energy_sample_at && tick>=energy_sample_at) {
  hw.EVENTS_RSSIEND=1; hw.RSSISAMPLE=periodic_energy(hw.FREQUENCY);
  raw_samples[hw.FREQUENCY]++; energy_sample_at=0;
 }
 if(energy_disabled_at && tick>=energy_disabled_at) { hw.EVENTS_DISABLED=1; hw.STATE=0; energy_disabled_at=0; }
}
static void k_busy_wait(unsigned us) {
 if(energy_scene) { periodic_radio_wait(us); return; }
 tick += us;
 if (hw.TASKS_RXEN) {
  hw.TASKS_RXEN=0; hw.STATE=2;
  if (!(raw_fail_ready && hw.FREQUENCY==(unsigned)raw_fail_channel)) hw.EVENTS_READY=1;
  assert(measured_n < sizeof(measured_ch)); measured_time[measured_n]=k_uptime_get(); measured_ch[measured_n++]=hw.FREQUENCY;
 }
 if (hw.TASKS_START) { hw.TASKS_START=0; if(hw.STATE==2) hw.STATE=1; }
 if (hw.TASKS_RSSISTART) {
  hw.TASKS_RSSISTART=0;
  if(raw_packet_end) { raw_ends++; hw.STATE=(hw.SHORTS & RADIO_SHORTS_END_START_Msk)?1:2; }
  if(hw.STATE==1 && !(raw_fail_sample && hw.FREQUENCY==(unsigned)raw_fail_channel)) {
   hw.EVENTS_RSSIEND=1; hw.RSSISAMPLE=raw_zero ? 0 : (hw.FREQUENCY==mock_channel ? 50 : 80);
   raw_samples[hw.FREQUENCY]++;
  }
 }
 if (hw.TASKS_DISABLE) { hw.TASKS_DISABLE=0; if (!fail_disable) { hw.EVENTS_DISABLED=1; hw.STATE=0; } }
}
static void k_msleep(unsigned ms) { tick += (uint64_t)ms*1000; }
static void k_sem_take(int *s,int wait) { (void)s; (void)wait; }
static void k_sem_give(int *s) { (void)s; }
static bool esb_pairing_is_active(void) { return pairing; }
static int esb_radio_operation_begin(void) { begins++; if (busy || owner) return -EBUSY; owner=true; return 0; }
static void esb_radio_operation_end(void) { assert(owner); assert(hw.STATE==1); owner=false; releases++; }
static int esb_radio_operation_stop(void) { assert(owner); if (fail_stop) return -EIO; hw.STATE=0; return 0; }
static uint8_t esb_get_receiver_channel(void) { return mock_channel; }
/* LOSS_SNAPSHOT_TYPES */
static struct esb_auto_loss_snapshot model;
static uint32_t received_rate[ESB_AUTO_LOSS_TRACKERS], gap_rate[ESB_AUTO_LOSS_TRACKERS];
static int64_t model_updated;
static bool model_eligible, fail_request;
static void esb_get_auto_loss_snapshot(struct esb_auto_loss_snapshot *out) {
 int64_t seconds=(k_uptime_get()-model_updated)/1000;
 for (unsigned i=0;i<ESB_AUTO_LOSS_TRACKERS;i++) {
  model.tracker[i].received += seconds*received_rate[i];
  model.tracker[i].gaps += seconds*gap_rate[i];
  if(received_rate[i] && seconds) model.ready_mask |= 1u<<i;
 }
 model_updated += seconds*1000;
 model.home=mock_channel; model.eligible=model_eligible && !busy && !owner && !pairing;
 *out=model;
}
static void esb_auto_loss_rebase(struct esb_auto_loss_snapshot *out) {
 esb_get_auto_loss_snapshot(out);
}
/* RSSI_SOURCE */
static void esb_set_addr_unified(void) { assert(owner); }
static int esb_radio_quiesce(void) { assert(owner); quiesces++; if (fail_disable) return -EIO; hw.STATE=0; return 0; }
static int esb_initialize_quiet(bool tx) { assert(!tx && owner && !hw.STATE); restores++; return fail_restore ? -EIO : 0; }
static int esb_start_rx(void) { if (fail_start) return -EIO; hw.STATE=1; hw.FREQUENCY=mock_channel; return 0; }
static int esb_home_beacon_send(const uint8_t frame[16]) {
 assert(owner && !hw.STATE && transmitted < 256);
 assert(frame[0]=='S' && frame[1]=='A' && frame[2]=='C');
 beacon_time[transmitted]=k_uptime_get(); beacon_ch[transmitted++]=mock_channel;
 return fail_tx ? -EIO : 0;
}
static int esb_auto_channel_request(uint8_t ch, uint8_t home, uint32_t token) {
 assert(!owner && hw.STATE==1); assert(restores && releases); assert(ch != home && ch <= 88 && !(ch & 1));
 assert(home==mock_channel && auto_channel_decision_valid(token));
 if (fail_request) return -EBUSY;
 migrations++; migrated_to=ch; return 0;
}
/* ACTUAL_SOURCE */
/* Leave enablement untouched so the first scenario sees the production BSS
 * default, not a fixture assignment standing in for boot behavior. */
static void initialize_fixture(void) {
 memset(&hw,0,sizeof(hw)); memset(&policy,0,sizeof(policy));
 reset_sweep();
 k_msgq_purge(&observations);
 tick=0; owner=busy=pairing=fail_disable=fail_restore=fail_start=fail_stop=fail_tx=false;
 radio_fault=entropy_fault=fail_entropy=false;
 begins=releases=restores=migrations=transmitted=quiesces=measured_n=seed_reads=0;
 memset(raw_samples,0,sizeof(raw_samples)); raw_ends=0; raw_fail_channel=-1;
 raw_fail_ready=raw_fail_sample=raw_packet_end=raw_zero=false;
 generation=0; initialized=0; random_state=4321; sequence=0;
 next_beacon=next_scan=0; observed_home=255; observed_generation=0;
 mock_channel=2; migrated_to=255; hw.STATE=1; hw.FREQUENCY=mock_channel;
 hw.PCNF1=32;
 energy_scene=0; energy_ready_at=energy_sample_at=energy_disabled_at=0;
 memset(&published,0,sizeof(published)); memset(&last_decision,0,sizeof(last_decision));
 tx_attempts=tx_completed=0;
 memset(&loss,0,sizeof(loss)); memset(&recovery,0,sizeof(recovery));
 memset(&model,0,sizeof(model)); memset(received_rate,0,sizeof(received_rate)); memset(gap_rate,0,sizeof(gap_rate));
 model.active_mask=1; model_eligible=true; received_rate[0]=100; model_updated=0;
 diagnostic=AC_MONITOR; interference_latched=0; next_evidence=0;
 completion_pending=false; migration_mask=recovery_seen=0; fail_request=false;
 warnings=0; warning_text[0]=0;
 const uint8_t identity[6]={2}; auto_channel_init(identity);
}
static void reset(void) {
 initialize_fixture();
 auto_channel_set_enabled(true);
}
static void run_until(int64_t until) {
 unsigned iterations=0;
 while (k_uptime_get()<until) {
  int64_t next=auto_step();
  assert(++iterations<10000);
  if (next>k_uptime_get()) tick=(uint64_t)MIN(next,until)*1000;
 }
 (void)auto_step();
}
static void receive_peer(uint8_t id, uint8_t home, uint16_t seq) {
 uint8_t frame[16], identity[6]={id}; ac_frame(frame,identity,home,seq);
 assert(auto_channel_receive(frame,sizeof(frame),0,home));
}
static void manual_opt_in(void) {
 initialize_fixture(); /* Must run before any reset or enable call. */
 received_rate[0]=799; gap_rate[0]=201;
 for (unsigned n=1;n<=70;n++) {
  receive_peer(1,2,n);
  run_until(n*1000);
 }
 struct auto_channel_snapshot status;
 auto_channel_get_snapshot(&status);
 assert(status.initialized && !status.enabled && !status.peer_count);
 assert(!begins && !transmitted && !measured_n && !migrations && !sweep.active);
 assert(!owner && hw.STATE==1 && hw.FREQUENCY==2);

 auto_channel_set_enabled(true);
 run_until(80000);
 assert(sweep.active && !measured_n && transmitted);
 run_until(81000);
 assert(measured_n && !migrations);
 unsigned stopped_begins=begins, stopped_tx=transmitted, stopped_samples=measured_n;
 uint32_t admitted_generation=auto_channel_generation();
 auto_channel_set_enabled(false);
 assert(!auto_channel_decision_valid(admitted_generation));
 for (unsigned n=1;n<=70;n++) {
  receive_peer(1,2,70+n);
  run_until(81000+n*1000);
 }
 auto_channel_get_snapshot(&status);
 assert(!status.enabled && !status.peer_count && !sweep.active);
 assert(begins==stopped_begins && transmitted==stopped_tx && measured_n==stopped_samples);
 assert(!migrations && !owner && hw.STATE==1 && hw.FREQUENCY==2);
 /* Exit with an explicit opt-in; another process must still start off. */
 auto_channel_set_enabled(true);
 puts("PASS boot off under high loss/peers; manual on scans; off blocks future RF admissions");
}
static void ready_loss(uint8_t home) {
 reset(); mock_channel=home; hw.FREQUENCY=home;
 received_rate[0]=799; gap_rate[0]=201;
 run_until(9999); assert(!sweep.active && !measured_n);
 run_until(10000); assert(sweep.active && !measured_n);
}
static void assert_cadence(void) {
 assert(transmitted>=15);
 for (unsigned i=0;i<transmitted;i++) {
  assert(beacon_ch[i]==mock_channel);
  if (i) assert(beacon_time[i]-beacon_time[i-1]>=2000 &&
                beacon_time[i]-beacon_time[i-1]<=4001);
 }
}
static void quiet_home(void) {
 reset(); run_until(70000);
 assert(!measured_n && !migrations && !sweep.active && !owner);
 assert(hw.FREQUENCY==mock_channel && hw.STATE==1); assert_cadence();
 int64_t min_spacing=INT64_MAX,max_spacing=0;
 unsigned away=0;
 for (unsigned i=1;i<transmitted;i++) {
  int64_t spacing=beacon_time[i]-beacon_time[i-1];
  min_spacing=MIN(min_spacing,spacing); max_spacing=MAX(max_spacing,spacing);
 }
 for (unsigned i=0;i<measured_n;i++) away+=measured_ch[i]!=mock_channel;
 for (unsigned i=0;i<transmitted;i++) away+=beacon_ch[i]!=mock_channel;
 printf("quiet host model: duration=%lldms home=%u beacons=%u RSSI_measurements=%u off_home_tunes=%u beacon_spacing=%lld..%lldms\n",
        (long long)k_uptime_get(),mock_channel,transmitted,measured_n,away,
        (long long)min_spacing,(long long)max_spacing);
 /* A peer never overrides the loss gate, even after the old startup delay. */
 reset();
 for (unsigned n=1;n<=3;n++) receive_peer(1,2,n);
 run_until(70000); assert(!measured_n && !sweep.active); assert_cadence();
}
static void loss_scans(void) {
 for (unsigned outside=0;outside<3;outside++) {
  uint8_t home=outside==2?99:(outside?90:2); ready_loss(home);
  while (!migrations && k_uptime_get()<50000) {
   int64_t next=auto_step(); if(next>k_uptime_get()) tick=(uint64_t)next*1000;
  }
  assert(migrations==1 && migrated_to==0 && !owner && hw.FREQUENCY==home);
  assert(last_decision.reference.visits==AC_SCAN_ROUNDS+AC_CONFIRM_ROUNDS);
  assert(last_decision.candidate.coverage_us==last_decision.reference.coverage_us);
  unsigned count[101]={0};
  for (unsigned i=0;i<measured_n;i++) {
   uint8_t ch=measured_ch[i]; assert(ch==home || (!(ch&1) && ch<=88)); count[ch]++;
  }
  for (unsigned ch=0;ch<=88;ch+=2) assert(count[ch]>=AC_SCAN_ROUNDS);
  assert(count[home]==last_decision.reference.visits);
  assert(count[migrated_to]==count[home]);
  assert(diagnostic==AC_WAIT_COMMIT); /* Admission is NOT a commit. */
  unsigned old_measured=measured_n; int64_t stopped=k_uptime_get();
  run_until(stopped+60000);
  assert(migrations==1 && measured_n==old_measured && !sweep.active && !warnings);
 }
 /* Both identity orderings may trial when their own loss qualifies. */
 for (unsigned identity=1;identity<=2;identity++) {
  reset(); policy.identity[0]=receiver_identity[0]=identity;
  for (unsigned n=1;n<=3;n++) receive_peer(3-identity,2,n);
  received_rate[0]=799; gap_rate[0]=201; run_until(10000);
  assert(sweep.active);
 }
}
static void invalid_frames(void) {
 reset(); run_until(60000);
 uint8_t good[17]={0}, frame[17], peer[6]={1}; ac_frame(good,peer,2,1);
 for (unsigned mode=0;mode<12;mode++) {
  memcpy(frame,good,sizeof(frame)); uint8_t len=16,pipe=0,home=2;
  switch(mode) {
   case 0: len=3; break;
   case 1: len=15; break;
   case 2: len=17; break;
   case 3: frame[3]=2; break;
   case 4: frame[13]=1; break;
   case 5: frame[14]=1; break;
   case 6: frame[15]=1; break;
   case 7: memset(frame+4,0,6); break;
   case 8: memcpy(frame+4,receiver_identity,6); break;
   case 9: pipe=1; break;
   case 10: home=3; break;
   case 11: home=frame[10]=101; break;
  }
  assert(auto_channel_is_beacon(frame,len));
  assert(auto_channel_receive(frame,len,pipe,home));
  assert(!observations.count);
 }
 assert(!auto_channel_receive(good,2,0,2));
 frame[0]='X'; assert(!auto_channel_receive(frame,16,0,2));
 (void)auto_step(); assert(!sweep.active && !measured_n);
 /* Queue owns the bytes, rather than retaining DMA-buffer pointers. */
 memcpy(frame,good,16); assert(auto_channel_receive(frame,16,0,2));
 memset(frame,0,16); (void)auto_step(); assert(ac_occupied(&policy,2,k_uptime_get()));
 reset();
 for (unsigned n=1;n<=100;n++) receive_peer(1,2,n);
 assert(observations.count==observations.capacity);
 (void)auto_step(); assert(!observations.count && !owner && !measured_n);
}
static void stale_and_excluded(void) {
 reset(); run_until(60000);
 for (unsigned n=1;n<=3;n++) receive_peer(1,2,n);
 uint32_t old=auto_channel_generation();
 auto_channel_set_enabled(false); auto_channel_set_enabled(true);
 assert(!auto_channel_decision_valid(old));
 run_until(70000); assert(!measured_n && !sweep.active);
 reset(); run_until(60000);
 for (unsigned n=1;n<=3;n++) receive_peer(1,2,n);
 mock_channel=4; hw.FREQUENCY=4; run_until(70000);
 assert(!measured_n && !sweep.active);
 reset(); run_until(60000);
 for (unsigned n=1;n<=3;n++) receive_peer(1,2,n);
 tick+=(uint64_t)AC_EXPIRY_MS*1000; (void)auto_step();
 assert(!sweep.active && !measured_n);
 for (unsigned mode=0;mode<4;mode++) {
  ready_loss(2); int64_t next=auto_step(); tick=(uint64_t)next*1000;
  (void)auto_step(); assert(measured_n && sweep.active);
  unsigned old_measured=measured_n, old_transmitted=transmitted;
  if (mode==0) auto_channel_set_enabled(false);
  if (mode==1) busy=true; /* Shared owner admission models OTA/collection. */
  if (mode==2) pairing=true;
  if (mode==3) { mock_channel=4; hw.FREQUENCY=4; }
  run_until(k_uptime_get()+9999);
  assert(measured_n==old_measured && !migrations && !sweep.active && !owner);
  if (mode<3) assert(transmitted==old_transmitted);
 }
}
static void radio_failures(void) {
 for (unsigned mode=0;mode<5;mode++) {
  reset(); (void)auto_step(); tick=(uint64_t)next_beacon*1000;
  if (mode==0) fail_tx=true;
  if (mode==1) fail_stop=true;
  if (mode==2) fail_restore=true;
  if (mode==3) fail_start=true;
  if (mode==4) fail_disable=true;
  (void)auto_step();
  assert(!migrations && !measured_n && !sweep.active);
  if (mode<2) {
   assert(!owner && !radio_fault && hw.STATE==1 && quiesces && restores);
  } else {
   assert(owner && radio_fault && !releases);
   unsigned old_begins=begins;
   run_until(k_uptime_get()+10000); assert(begins==old_begins);
  }
 }
 reset(); fail_entropy=true; auto_thread(NULL,NULL,NULL);
 assert(entropy_fault && seed_reads==1 && !begins && !owner && !transmitted);
 assert(!auto_channel_decision_valid(auto_channel_generation()));
}
static void raw_measurement_failures(void) {
 for(unsigned phase=0;phase<2;phase++) {
  ready_loss(18); tick=(uint64_t)next_scan*1000;
  raw_fail_channel=sweep.order[sweep.candidate];
  raw_fail_ready=phase==0; raw_fail_sample=phase==1;
  unsigned old_restores=restores;
  (void)auto_step();
  assert(!migrations && !sweep.active && !owner && !radio_fault && restores>old_restores && !loss.valid);
  for(unsigned ch=0;ch<101;ch++) assert(scores[ch]==-1);
  raw_fail_ready=raw_fail_sample=false;
  run_until(k_uptime_get()+9000); assert(!sweep.active && !migrations);
 }
 ready_loss(18); tick=(uint64_t)next_scan*1000; raw_packet_end=true;
 (void)auto_step(); assert(sweep.active && raw_ends && !radio_fault);
 ready_loss(18); tick=(uint64_t)next_scan*1000; raw_zero=true;
 uint8_t ch=sweep.order[sweep.candidate];
 (void)auto_step(); assert(sweep.active && scores[ch]==0 && !radio_fault);
 ready_loss(18); tick=(uint64_t)next_scan*1000; raw_fail_sample=true;
 raw_fail_channel=sweep.order[sweep.candidate]; fail_restore=true;
 (void)auto_step(); assert(owner && radio_fault && !migrations && !sweep.active);
}
static void loss_windows(void) {
 /* Strict threshold: 19.9%, 20.0%, 20.1%; require a full ten seconds. */
 for (unsigned gaps=199;gaps<=201;gaps++) {
  reset(); received_rate[0]=1000-gaps; gap_rate[0]=gaps;
  run_until(9999); assert(!sweep.active && !measured_n);
  run_until(10000); assert(sweep.active==(gaps>200));
 }
 /* Packet-weighting across contributors, not arithmetic mean of percentages. */
 reset(); model.active_mask=3; received_rate[0]=990;
 received_rate[1]=1; gap_rate[1]=9;
 run_until(15000); assert(!sweep.active && !measured_n);
 reset(); model.active_mask=3; received_rate[0]=1;
 received_rate[1]=798; gap_rate[1]=201;
 run_until(10000); assert(sweep.active);
 /* Sparse windows expire UNKNOWN; never extend until 100 samples appear. */
 reset(); received_rate[0]=6; gap_rate[0]=3;
 run_until(30000); assert(!sweep.active && !measured_n);
 reset(); received_rate[0]=7; gap_rate[0]=3;
 run_until(10000); assert(sweep.active);
 /* Every current contributor needs unique normal traffic in the window. */
 reset(); model.active_mask=3; received_rate[0]=79; gap_rate[0]=21;
 run_until(30000); assert(!sweep.active && !measured_n);
 /* A whole idle window is UNKNOWN; first returning gap is discarded. */
 reset(); run_until(9000);
 received_rate[0]=gap_rate[0]=0; run_until(19000);
 received_rate[0]=79; gap_rate[0]=21; model.tracker[0].gaps+=10000;
 run_until(29999); assert(!sweep.active && !measured_n);
 run_until(30000); assert(sweep.active);
 /* Reset, restart, member changes, exclusion and scheduling discontinuity
  * all require a fresh baseline and ten fresh seconds. */
 for (unsigned mode=0;mode<8;mode++) {
  reset(); received_rate[0]=79; gap_rate[0]=21; run_until(9000);
  if (mode==0) model.reset_epoch++;
  if (mode==1) model.tracker[0].restarts++;
  if (mode==2) { model.active_mask=3; received_rate[1]=79; gap_rate[1]=21; }
  if (mode==3) model_eligible=false;
  if (mode==4) { model.tracker[0].received=0; model.tracker[0].gaps=0; }
  if (mode==5) { tick=12000000; (void)auto_step(); }
  if (mode==6) auto_channel_set_enabled(true);
  if (mode==7) { mock_channel=5; hw.FREQUENCY=5; }
  int64_t baseline=mode==5?12000:(mode>=6?9000:10000);
  run_until(baseline);
  if (mode==3) { model_eligible=true; baseline=11000; run_until(baseline); }
  run_until(baseline+9999); assert(!sweep.active && !measured_n);
  run_until(baseline+10000); assert(sweep.active);
 }
 /* Weighted temporal aggregation too: five tiny 90%-loss intervals do not
  * outweigh five high-volume clean intervals. */
 reset();
 struct esb_auto_loss_snapshot s={.active_mask=1,.ready_mask=1,.eligible=true,.home=2};
 assert(loss_sample(&s,0)==-1);
 for (unsigned n=1;n<=10;n++) {
  s.tracker[0].received += n<=5?1000:1;
  s.tracker[0].gaps += n<=5?0:9;
  int outcome=loss_sample(&s,n*1000);
  assert(outcome==(n==10?0:-1));
 }
 /* A sleepy contributor is valid if it has unique activity anywhere inside
  * the complete window; it need not contribute in every one-second bin. */
 memset(&loss,0,sizeof(loss)); memset(&s,0,sizeof(s));
 s.active_mask=3; s.ready_mask=3; s.eligible=true; s.home=2;
 assert(loss_sample(&s,0)==-1);
 for (unsigned n=1;n<=10;n++) {
  s.tracker[0].received+=79; s.tracker[0].gaps+=21;
  if (n==5) s.tracker[1].received++;
  int outcome=loss_sample(&s,n*1000);
  assert(outcome==(n==10?1:-1));
 }
 /* One real normal receipt plus 39 inferred gaps every two seconds is
  * sufficient evidence; its empty alternate bins must not starve admission. */
 memset(&loss,0,sizeof(loss)); memset(&s,0,sizeof(s));
 s.active_mask=1; s.ready_mask=1; s.eligible=true; s.home=2;
 assert(loss_sample(&s,0)==-1);
 for (unsigned n=1;n<=10;n++) {
  if (!(n&1)) { s.tracker[0].received++; s.tracker[0].gaps+=39; }
  assert(loss_sample(&s,n*1000)==(n==10?1:-1));
 }
 /* Ten observations are not ten seconds if their endpoints span 9900ms. */
 memset(&loss,0,sizeof(loss)); memset(&s.tracker,0,sizeof(s.tracker));
 assert(loss_sample(&s,0)==-1);
 for (unsigned n=1;n<=10;n++) {
  s.tracker[0].received+=79; s.tracker[0].gaps+=21;
  assert(loss_sample(&s,n*1000-100)==-1);
 }
 s.tracker[0].received+=79; s.tracker[0].gaps+=21;
 assert(loss_sample(&s,10900)==1);
 /* Repeated late 1500ms samples cannot accumulate a mislabeled 15s window. */
 memset(&loss,0,sizeof(loss)); memset(&s.tracker,0,sizeof(s.tracker));
 assert(loss_sample(&s,0)==-1);
 for (unsigned n=1;n<=10;n++) {
  s.tracker[0].received+=79; s.tracker[0].gaps+=21;
  assert(loss_sample(&s,n*1500)==-1);
 }
 assert(!loss.count);
 reset(); received_rate[0]=79; gap_rate[0]=21; (void)auto_step();
 tick=1500000; (void)auto_step(); assert(next_evidence==2000);
 tick=3000000; (void)auto_step(); assert(next_evidence==4000 && !loss.count);
}

static void finish_scan(void) {
 int64_t deadline=k_uptime_get()+100000;
 while (sweep.active && k_uptime_get()<deadline) {
  int64_t next=auto_step(); if(next>k_uptime_get()) tick=(uint64_t)next*1000;
 }
 assert(!sweep.active);
}
static void scan_learning_frozen(void) {
 ready_loss(2);
 loss.count=0; /* Scan-induced fencing cannot revoke the prior admission. */
 run_until(20000);
 assert(sweep.active && measured_n && !migrations);
 finish_scan();
 assert(migrations==1);
}
static void start_trial(void) {
 ready_loss(2); finish_scan();
 assert(migrations==1 && diagnostic==AC_WAIT_COMMIT);
}
static void commit_trial(bool success) {
 model.ready_mask=0; model.reset_epoch++;
 if (success) { mock_channel=migrated_to; hw.FREQUENCY=mock_channel; }
 auto_channel_migration_complete(auto_channel_generation(),requested_home,mock_channel,success);
 (void)auto_step();
 assert(diagnostic==(success?AC_WAIT_RECOVERY:AC_MONITOR));
}
static void layout_postcheck_boundary(void) {
 start_trial(); commit_trial(true); run_until(next_evidence);
 assert(diagnostic==AC_POSTCHECK);
 run_until(k_uptime_get()+7000); assert(!warnings);
 model.cap_pending=true; model.layout_epoch++; model.reset_epoch++; model.ready_mask=0;
 run_until(next_evidence); assert(diagnostic==AC_WAIT_RECOVERY && !warnings);
 run_until(k_uptime_get()+3000); assert(!warnings && diagnostic==AC_WAIT_RECOVERY);
 model.cap_pending=false;
 while(diagnostic==AC_WAIT_RECOVERY) run_until(next_evidence);
 assert(diagnostic==AC_POSTCHECK);
 int64_t post=k_uptime_get();
 run_until(post+9999); assert(!warnings);
 run_until(post+10000); assert(warnings==1 && interference_latched);
}
static void trial_lifecycle(void) {
 /* Rejected request and failed actual commit both require new evidence. */
 ready_loss(2); fail_request=true; finish_scan();
 int64_t stopped=k_uptime_get();
 assert(!migrations && diagnostic==AC_MONITOR);
 run_until(stopped+9000); assert(!sweep.active && !warnings);
 start_trial(); commit_trial(false); stopped=k_uptime_get();
 run_until(stopped+9999); assert(!sweep.active && !warnings);
 run_until(stopped+10000); assert(sweep.active);

 /* Successful commit waits indefinitely for NORMAL accepted data. PING,
  * duplicates and last_packet_time cannot affect these counters. */
 start_trial(); received_rate[0]=gap_rate[0]=0; commit_trial(true);
 unsigned measured=measured_n; run_until(k_uptime_get()+60000);
 assert(diagnostic==AC_WAIT_RECOVERY && !warnings && migrations==1 && measured_n==measured);
 model.tracker[0].received++; model.tracker[0].gaps+=100000; model.ready_mask=1;
 run_until(next_evidence); assert(diagnostic==AC_POSTCHECK);
 received_rate[0]=799; gap_rate[0]=201;
 int64_t post=k_uptime_get();
 run_until(post+9999); assert(!warnings && !interference_latched);
 run_until(post+10000);
 assert(warnings==1 && interference_latched && migrations==1);
 assert(strstr(warning_text,"Possible host USB/RF interference") && strstr(warning_text,"USB extension"));
 assert(strstr(auto_channel_status(),"stopped"));
 unsigned tx=transmitted, radio_begins=begins;
 run_until(k_uptime_get()+180000);
 assert(warnings==1 && transmitted==tx && begins==radio_begins && measured_n==measured);
 auto_channel_set_enabled(false); mock_channel=6; hw.FREQUENCY=6;
 run_until(k_uptime_get()+10000); assert(interference_latched && warnings==1);
 auto_channel_set_enabled(true); received_rate[0]=100; gap_rate[0]=0;
 run_until(k_uptime_get()+11000); assert(!interference_latched && transmitted>tx);

 /* Exactly 20% is a clean post-window: ignore the first migration gap and
  * resume monitoring only after the existing 120s residence. */
 start_trial(); commit_trial(true); received_rate[0]=800; gap_rate[0]=200;
 model.tracker[0].gaps+=100000; run_until(next_evidence);
 assert(diagnostic==AC_POSTCHECK);
 post=k_uptime_get(); run_until(post+10000);
 assert(diagnostic==AC_MONITOR && !warnings && !interference_latched);
 received_rate[0]=799; gap_rate[0]=201;
 run_until(policy.cooldown-1); assert(migrations==1 && !sweep.active);
 run_until(policy.cooldown+1000); assert(sweep.active);

 /* A missing original participant cannot silently disappear from the
  * postcheck denominator when membership shrinks. */
 start_trial(); model.active_mask=3; model.tracker[1].received=100;
 migration_mask=3; commit_trial(true);
 run_until(k_uptime_get()+20000); assert(diagnostic==AC_WAIT_RECOVERY);
 model.active_mask=1; run_until(next_evidence); assert(diagnostic==AC_WAIT_RECOVERY);
 run_until(k_uptime_get()+20000); assert(diagnostic==AC_WAIT_RECOVERY && !warnings);

 /* Old actual completions cannot survive explicit disable/re-enable or a
  * manual generation change. */
 start_trial(); uint32_t stale=auto_channel_generation();
 auto_channel_set_enabled(false); auto_channel_set_enabled(true);
 auto_channel_migration_complete(stale,2,migrated_to,true);
 received_rate[0]=100; gap_rate[0]=0; (void)auto_step();
 assert(diagnostic==AC_MONITOR && !completion_pending && !warnings);
 run_until(k_uptime_get()+11000); assert(!sweep.active);
}
static void decision_history(void) {
 start_trial();
 struct auto_channel_snapshot before, after;
 auto_channel_get_snapshot(&before);
 assert(before.decision.valid && before.decision.home==before.home);
 assert(before.decision.target==migrated_to);
 assert(before.decision.reference.peak==50 && before.decision.candidate.peak==80);
 assert(before.decision.reference.visits==before.decision.candidate.visits);
 commit_trial(true);
 auto_channel_get_snapshot(&after);
 assert(after.home==migrated_to && after.home!=before.home);
 assert(after.decision.valid && after.decision.home==before.home);
 assert(after.decision.target==before.decision.target);
 assert(after.decision.time_ms==before.decision.time_ms);
 assert(after.decision.generation==before.decision.generation);
 assert(after.decision.reference.peak==before.decision.reference.peak);
 assert(after.decision.reference.quiet_floor==before.decision.reference.quiet_floor);
 assert(after.decision.reference.coverage_us==before.decision.reference.coverage_us);
 assert(after.decision.candidate.peak==before.decision.candidate.peak);
 assert(after.decision.candidate.coverage_us==before.decision.candidate.coverage_us);
 assert(after.decision.reason==before.decision.reason);
 auto_channel_set_enabled(false);
 auto_channel_get_snapshot(&after);
 assert(!after.enabled && !after.decision.valid && !after.peer_count);
 assert(after.generation!=before.generation);
}
static void periodic_energy_decisions(void) {
 for(unsigned scene=1;scene<=2;scene++) {
  reset(); energy_scene=scene; mock_channel=88; hw.FREQUENCY=88; hw.PCNF1=72;
  received_rate[0]=799; gap_rate[0]=201;
  while(!last_decision.valid && k_uptime_get()<75000) {
   int64_t next=auto_step();
   if(next>k_uptime_get()) tick=(uint64_t)next*1000;
  }
  assert(last_decision.valid && !owner && hw.FREQUENCY==88);
  if(scene==1) {
   assert(!migrations); /* Never trial busy52 merely because its visits alias. */
  } else {
   assert(migrations==1 && migrated_to==74);
   assert(last_decision.reference.peak==45 && last_decision.reference.quiet_floor==96);
   assert(last_decision.candidate.peak==96);
   assert(last_decision.reference.visits==last_decision.candidate.visits);
   assert(last_decision.reference.coverage_us==last_decision.candidate.coverage_us);
  }
 }
}
int main(int argc, char **argv) {
 manual_opt_in();
 if (argc==2 && !strcmp(argv[1],"--boot-only")) return 0;
 quiet_home(); loss_scans(); invalid_frames(); stale_and_excluded(); radio_failures();
 loss_windows(); trial_lifecycle(); raw_measurement_failures();
 scan_learning_frozen(); layout_postcheck_boundary();
 decision_history();
 periodic_energy_decisions();
 puts("PASS actual auto worker: loss window, short scan, commit/recovery/postcheck latch, quiet HOME cadence and lifecycle faults");
 return 0;
}
