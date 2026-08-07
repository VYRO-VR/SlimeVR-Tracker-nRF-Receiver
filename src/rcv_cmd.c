/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
#include "rcv_cmd.h"

#include "build_defines.h"
#include "connection/esb.h"
#include "connection/rssi_scan.h"
#include "data_collect.h"
#include "globals.h"
#include "system/system.h"

#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(rcv_cmd, LOG_LEVEL_INF);

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER)

static atomic_t rssi_scan_busy = ATOMIC_INIT(0);

/* Async completion ACK for tracker channel-all (seq/opcode from STARTED request). */
static atomic_t ch_hid_pending = ATOMIC_INIT(0);
static uint8_t ch_hid_seq;
static uint8_t ch_hid_opcode;
static rcv_cmd_async_ack_fn async_ack_fn;

enum pending_reset {
	PENDING_RESET_NONE = 0,
	PENDING_RESET_REBOOT,
	PENDING_RESET_DFU_UF2,
	PENDING_RESET_DFU_OTA,
};

static atomic_t pending_reset = ATOMIC_INIT(PENDING_RESET_NONE);

static void rssi_scan_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	rssi_scan_run_and_print();
	atomic_set(&rssi_scan_busy, 0);
}

static K_WORK_DEFINE(rssi_scan_work, rssi_scan_work_handler);

/* Own queue so multi-second scan cannot stall HID send_report on system WQ. */
static K_THREAD_STACK_DEFINE(rssi_wq_stack, 1024);
static struct k_work_q rssi_wq;

static int rssi_wq_init(void)
{
	k_work_queue_init(&rssi_wq);
	k_work_queue_start(&rssi_wq, rssi_wq_stack, K_THREAD_STACK_SIZEOF(rssi_wq_stack),
			   RSSI_WQ_PRIORITY, NULL);
	k_thread_name_set(&rssi_wq.thread, "rssi_wq");
	return 0;
}

SYS_INIT(rssi_wq_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void reset_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int kind = (int)atomic_get(&pending_reset);
	atomic_set(&pending_reset, PENDING_RESET_NONE);
	switch (kind) {
	case PENDING_RESET_REBOOT:
		sys_skip_dfu_marker();
		sys_request_system_reboot();
		break;
	case PENDING_RESET_DFU_UF2:
		sys_enter_dfu(false);
		break;
	case PENDING_RESET_DFU_OTA:
		sys_enter_dfu(true);
		break;
	default:
		break;
	}
}

static K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);

static void schedule_reset(enum pending_reset kind)
{
	atomic_set(&pending_reset, kind);
	k_work_schedule(&reset_work, K_MSEC(50));
}

void rcv_cmd_set_async_ack(rcv_cmd_async_ack_fn fn)
{
	async_ack_fn = fn;
}

uint8_t rcv_cmd_pair(uint8_t count)
{
	if (count == 0) {
		esb_start_pairing();
	} else {
		esb_start_pairing_with_count(count);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_exit_pair(void)
{
	esb_finish_pair();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_clear(void)
{
	esb_clear();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_add(uint64_t addr, int8_t *slot_out)
{
	if (addr == 0) {
		return RCV_HID_ST_EINVAL;
	}
	int slot = esb_add_pair(addr, true);
	if (slot >= 0) {
		if (slot_out) {
			*slot_out = (int8_t)slot;
		}
		return RCV_HID_ST_OK;
	}
	if (slot == -ENOSPC) {
		return RCV_HID_ST_ENOSPC;
	}
	return RCV_HID_ST_EINVAL;
}

uint8_t rcv_cmd_remove(void)
{
	esb_pop_pair();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_list(void)
{
	printk("Stored devices:\n");
	for (uint8_t i = 0; i < stored_trackers; i++) {
		printk("%012llX\n", stored_tracker_addr[i]);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_channel_set(uint8_t channel)
{
	if (channel < 1 || channel > 100) {
		return RCV_HID_ST_EINVAL;
	}
	esb_set_receiver_channel(channel);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_channel_clear(void)
{
	esb_clear_receiver_channel();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_rssi_scan(void)
{
	if (!atomic_cas(&rssi_scan_busy, 0, 1)) {
		return RCV_HID_ST_EBUSY;
	}
	k_work_submit_to_queue(&rssi_wq, &rssi_scan_work);
	return RCV_HID_ST_STARTED;
}

uint8_t rcv_cmd_info(void)
{
	printk(CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER " " CONFIG_SLIMEVR_USB_DEVICE_PRODUCT "\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nDevice address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);

	uint8_t current_channel = esb_get_receiver_channel();
	if (current_channel != 0xFF && current_channel <= 100) {
		printk("RF Channel: %u (custom)\n", current_channel);
	} else {
		printk("RF Channel: %u (default)\n", CONFIG_RADIO_RF_CHANNEL);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_uptime(void)
{
	int64_t uptime = k_ticks_to_us_floor64(k_uptime_ticks());

	uint32_t days = uptime / 86400000000;
	uptime %= 86400000000;
	uint8_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime %= 1000;

	printk("Uptime: %u.%02u:%02u:%02u.%03u,%03u\n", days, hours, minutes, seconds, milliseconds,
	       microseconds);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_stats(uint32_t duration_seconds)
{
	if (duration_seconds == 0) {
		esb_toggle_stats_detailed();
	} else {
		esb_set_stats_detailed(duration_seconds);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_resetstats(void)
{
	esb_reset_all_stats();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_collect_start(uint8_t tracker_id)
{
#ifndef CONFIG_DATA_COLLECT
	ARG_UNUSED(tracker_id);
	return RCV_HID_ST_ENOTSUP;
#else
	if (tracker_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	data_collect_start(tracker_id);
	esb_send_remote_command(tracker_id, ESB_PONG_FLAG_DATA_COLLECT_ON);
	return RCV_HID_ST_OK;
#endif
}

uint8_t rcv_cmd_collect_stop(void)
{
#ifndef CONFIG_DATA_COLLECT
	return RCV_HID_ST_ENOTSUP;
#else
	if (data_collect_is_active()) {
		uint8_t tid = data_collect_get_target_id();
		data_collect_stop();
		esb_send_remote_command(tid, ESB_PONG_FLAG_DATA_COLLECT_OFF);
	}
	return RCV_HID_ST_OK;
#endif
}

uint8_t rcv_cmd_reboot(void)
{
	schedule_reset(PENDING_RESET_REBOOT);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_dfu(bool ota)
{
#if !DFU_EXISTS
	ARG_UNUSED(ota);
	return RCV_HID_ST_ENOTSUP;
#else
	schedule_reset(ota ? PENDING_RESET_DFU_OTA : PENDING_RESET_DFU_UF2);
	return RCV_HID_ST_OK;
#endif
}

static void fill_ack(uint8_t ack[RCV_HID_CMD_LEN], uint8_t seq, uint8_t opcode, uint8_t status);

/* ---- HID remote completion (STARTED → OK/ENOENT after tracker confirm) ---- */
#define REMOTE_HID_CONFIRM_TIMEOUT_MS 5000

static atomic_t remote_hid_pending = ATOMIC_INIT(0);
static uint8_t remote_hid_seq;
static uint8_t remote_hid_opcode;
static uint8_t remote_hid_flag;
static atomic_t remote_hid_expected = ATOMIC_INIT(0);
static atomic_t remote_hid_confirmed = ATOMIC_INIT(0);

enum remote_hid_job {
	REMOTE_HID_JOB_NONE = 0,
	REMOTE_HID_JOB_FLAG_ALL,
	REMOTE_HID_JOB_SENS_AUTO_ALL,
};

static enum remote_hid_job remote_hid_job;
static uint8_t remote_hid_job_flag;
static uint8_t remote_hid_job_axis;
static uint16_t remote_hid_job_rev;

static void remote_hid_timeout_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(remote_hid_timeout_work, remote_hid_timeout_handler);
static void remote_hid_job_handler(struct k_work *work);
static K_WORK_DEFINE(remote_hid_job_work, remote_hid_job_handler);

static void remote_hid_send_completion(uint8_t status)
{
	if (!async_ack_fn) {
		return;
	}
	uint8_t ack[RCV_HID_CMD_LEN];
	fill_ack(ack, remote_hid_seq, remote_hid_opcode, status);
	LOG_INF("HID remote complete seq=%u opcode=0x%02X status=%u", remote_hid_seq, remote_hid_opcode,
		status);
	async_ack_fn(ack);
}

static void remote_hid_abort(void)
{
	atomic_set(&remote_hid_pending, 0);
	remote_hid_job = REMOTE_HID_JOB_NONE;
	(void)k_work_cancel_delayable(&remote_hid_timeout_work);
}

static void remote_hid_finish(uint8_t status)
{
	if (!atomic_cas(&remote_hid_pending, 1, 0)) {
		return;
	}
	(void)k_work_cancel_delayable(&remote_hid_timeout_work);
	remote_hid_job = REMOTE_HID_JOB_NONE;
	remote_hid_send_completion(status);
}

static void remote_hid_arm_expected(uint32_t mask)
{
	atomic_set(&remote_hid_confirmed, 0);
	atomic_set(&remote_hid_expected, (atomic_val_t)mask);
	if (mask == 0) {
		remote_hid_finish(RCV_HID_ST_ENOENT);
		return;
	}
	(void)k_work_reschedule(&remote_hid_timeout_work, K_MSEC(REMOTE_HID_CONFIRM_TIMEOUT_MS));
}

static void remote_hid_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!atomic_get(&remote_hid_pending)) {
		return;
	}
	uint32_t expected = (uint32_t)atomic_get(&remote_hid_expected);
	uint32_t confirmed = (uint32_t)atomic_get(&remote_hid_confirmed);
	LOG_WRN("HID remote confirm timeout opcode=0x%02X confirmed=0x%08X expected=0x%08X",
		remote_hid_opcode, confirmed, expected);
	remote_hid_finish(RCV_HID_ST_ENOENT);
}

static void remote_hid_job_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!atomic_get(&remote_hid_pending)) {
		return;
	}

	uint32_t mask = 0;
	switch (remote_hid_job) {
	case REMOTE_HID_JOB_FLAG_ALL:
		mask = esb_send_remote_command_all(remote_hid_job_flag);
		break;
	case REMOTE_HID_JOB_SENS_AUTO_ALL:
		mask = esb_send_remote_command_sens_auto_all(remote_hid_job_axis, remote_hid_job_rev);
		break;
	default:
		remote_hid_finish(RCV_HID_ST_EINVAL);
		return;
	}
	remote_hid_arm_expected(mask);
}

static void on_remote_confirm(uint8_t tracker_id, uint8_t flag)
{
	if (!atomic_get(&remote_hid_pending)) {
		return;
	}
	if (flag != remote_hid_flag || tracker_id >= MAX_TRACKERS) {
		return;
	}

	uint32_t expected = (uint32_t)atomic_get(&remote_hid_expected);
	uint32_t bit = 1u << tracker_id;
	if (expected == 0 || (expected & bit) == 0) {
		return;
	}

	atomic_val_t old = atomic_or(&remote_hid_confirmed, (atomic_val_t)bit);
	uint32_t confirmed = (uint32_t)old | bit;
	if ((confirmed & expected) == expected) {
		remote_hid_finish(RCV_HID_ST_OK);
	}
}

static uint8_t remote_hid_begin(uint8_t seq, uint8_t opcode, uint8_t flag)
{
	if (!atomic_cas(&remote_hid_pending, 0, 1)) {
		return RCV_HID_ST_EBUSY;
	}
	remote_hid_seq = seq;
	remote_hid_opcode = opcode;
	remote_hid_flag = flag;
	atomic_set(&remote_hid_expected, 0);
	atomic_set(&remote_hid_confirmed, 0);
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_flag_hid(uint8_t seq, uint8_t opcode, uint8_t target_id, uint8_t pong_flag)
{
	if (!rcv_hid_opcode_is_pong_flag(pong_flag)) {
		return RCV_HID_ST_EINVAL;
	}

	uint8_t st = remote_hid_begin(seq, opcode, pong_flag);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}

	if (target_id == RCV_HID_TARGET_ALL) {
		/* Defer 1s active-scan so HID IN FIFO keeps draining. */
		remote_hid_job = REMOTE_HID_JOB_FLAG_ALL;
		remote_hid_job_flag = pong_flag;
		k_work_submit(&remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}

	/* Arm before queue so a fast confirm cannot race expected==0. */
	remote_hid_arm_expected(1u << target_id);
	esb_send_remote_command(target_id, pong_flag);
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_sens_set_hid(uint8_t seq, uint8_t opcode, uint8_t target_id, float x,
					  float y, float z)
{
	uint8_t st = remote_hid_begin(seq, opcode, ESB_PONG_FLAG_SENS_SET);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}

	if (target_id == RCV_HID_TARGET_ALL) {
		uint32_t mask = 0;
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			mask |= (1u << i);
		}
		remote_hid_arm_expected(mask);
		if (mask == 0) {
			return RCV_HID_ST_STARTED;
		}
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			esb_send_remote_command_sens(i, x, y, z);
		}
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	remote_hid_arm_expected(1u << target_id);
	esb_send_remote_command_sens(target_id, x, y, z);
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_sens_auto_hid(uint8_t seq, uint8_t opcode, uint8_t target_id,
					   uint8_t axis, uint16_t revolutions)
{
	if (axis > 2) {
		return RCV_HID_ST_EINVAL;
	}

	uint8_t st = remote_hid_begin(seq, opcode, ESB_PONG_FLAG_SENS_AUTO);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}

	if (target_id == RCV_HID_TARGET_ALL) {
		remote_hid_job = REMOTE_HID_JOB_SENS_AUTO_ALL;
		remote_hid_job_axis = axis;
		remote_hid_job_rev = revolutions;
		k_work_submit(&remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	remote_hid_arm_expected(1u << target_id);
	if (!esb_send_remote_command_sens_auto(target_id, axis, revolutions)) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	return RCV_HID_ST_STARTED;
}

static void arm_ch_hid_completion(uint8_t seq, uint8_t opcode)
{
	ch_hid_seq = seq;
	ch_hid_opcode = opcode;
	atomic_set(&ch_hid_pending, 1);
}

static void on_tracker_channel_done(bool success)
{
	if (!atomic_cas(&ch_hid_pending, 1, 0)) {
		return;
	}
	if (!async_ack_fn) {
		return;
	}

	uint8_t ack[RCV_HID_CMD_LEN];
	fill_ack(ack, ch_hid_seq, ch_hid_opcode, success ? RCV_HID_ST_OK : RCV_HID_ST_ENOENT);
	async_ack_fn(ack);
}

static int map_esb_channel_err(int err)
{
	if (err == 0) {
		return RCV_HID_ST_STARTED;
	}
	if (err == -EBUSY) {
		return RCV_HID_ST_EBUSY;
	}
	return RCV_HID_ST_EINVAL;
}

uint8_t rcv_cmd_tracker_channel_all(uint8_t channel)
{
	if (channel < 1 || channel > 100) {
		return RCV_HID_ST_EINVAL;
	}
	return (uint8_t)map_esb_channel_err(esb_set_all_trackers_channel(channel));
}

uint8_t rcv_cmd_tracker_channel_clear_all(void)
{
	return (uint8_t)map_esb_channel_err(esb_clear_all_trackers_channel());
}

uint8_t rcv_cmd_remote_flag(uint8_t target_id, uint8_t pong_flag)
{
	if (!rcv_hid_opcode_is_pong_flag(pong_flag)) {
		return RCV_HID_ST_EINVAL;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		(void)esb_send_remote_command_all(pong_flag);
		return RCV_HID_ST_QUEUED;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	esb_send_remote_command(target_id, pong_flag);
	return RCV_HID_ST_QUEUED;
}

uint8_t rcv_cmd_remote_sens_set(uint8_t target_id, float x, float y, float z)
{
	if (target_id == RCV_HID_TARGET_ALL) {
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			esb_send_remote_command_sens(i, x, y, z);
		}
		return RCV_HID_ST_QUEUED;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	esb_send_remote_command_sens(target_id, x, y, z);
	return RCV_HID_ST_QUEUED;
}

uint8_t rcv_cmd_remote_sens_auto(uint8_t target_id, uint8_t axis, uint16_t revolutions)
{
	if (axis > 2) {
		return RCV_HID_ST_EINVAL;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		(void)esb_send_remote_command_sens_auto_all(axis, revolutions);
		return RCV_HID_ST_QUEUED;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	if (!esb_send_remote_command_sens_auto(target_id, axis, revolutions)) {
		return RCV_HID_ST_EINVAL;
	}
	return RCV_HID_ST_QUEUED;
}

static void fill_ack(uint8_t ack[RCV_HID_CMD_LEN], uint8_t seq, uint8_t opcode, uint8_t status)
{
	memset(ack, 0, RCV_HID_CMD_LEN);
	ack[0] = RCV_HID_TYPE_CMD_ACK;
	ack[1] = seq;
	ack[2] = opcode;
	ack[3] = status;
}

static int rcv_cmd_register_esb_cbs(void)
{
	esb_set_channel_change_done_cb(on_tracker_channel_done);
	esb_set_remote_confirm_cb(on_remote_confirm);
	return 0;
}

SYS_INIT(rcv_cmd_register_esb_cbs, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

bool rcv_cmd_process_hid(const uint8_t *buf, size_t len, uint8_t ack_out[RCV_HID_CMD_LEN])
{
	if (buf == NULL || ack_out == NULL || len < 4 || buf[0] != RCV_HID_TYPE_CMD) {
		return false;
	}

	uint8_t seq = buf[1];
	uint8_t opcode = buf[2];
	const uint8_t *args = &buf[4];
	size_t args_len = (len > 4) ? (len - 4) : 0;
	if (args_len > 12) {
		args_len = 12;
	}

	LOG_INF("HID cmd seq=%u opcode=0x%02X args_len=%u", seq, opcode, (unsigned)args_len);

	uint8_t status = RCV_HID_ST_EINVAL;
	int8_t slot = -1;

	if (rcv_hid_opcode_is_pong_flag(opcode)) {
		uint8_t target = (args_len >= 1) ? args[0] : RCV_HID_TARGET_ALL;
		if (opcode == ESB_PONG_FLAG_SENS_SET) {
			if (args_len < 7) {
				status = RCV_HID_ST_EINVAL;
			} else {
				int16_t xi = (int16_t)sys_get_le16(&args[1]);
				int16_t yi = (int16_t)sys_get_le16(&args[3]);
				int16_t zi = (int16_t)sys_get_le16(&args[5]);
				status = rcv_cmd_remote_sens_set_hid(seq, opcode, target, xi / 100.0f,
								     yi / 100.0f, zi / 100.0f);
			}
		} else if (opcode == ESB_PONG_FLAG_SENS_AUTO) {
			if (args_len < 4) {
				status = RCV_HID_ST_EINVAL;
			} else {
				uint8_t axis = args[1];
				uint16_t rev = sys_get_le16(&args[2]);
				status = rcv_cmd_remote_sens_auto_hid(seq, opcode, target, axis, rev);
			}
		} else if (opcode == ESB_PONG_FLAG_SET_CHANNEL ||
			   opcode == ESB_PONG_FLAG_CLEAR_CHANNEL) {
			status = RCV_HID_ST_EINVAL;
		} else {
			status = rcv_cmd_remote_flag_hid(seq, opcode, target, opcode);
		}
	} else {
		switch (opcode) {
		case RCV_HID_OP_NOP:
			status = RCV_HID_ST_OK;
			break;
		case RCV_HID_OP_PAIR:
			status = rcv_cmd_pair((args_len >= 1) ? args[0] : 0);
			break;
		case RCV_HID_OP_EXIT_PAIR:
			status = rcv_cmd_exit_pair();
			break;
		case RCV_HID_OP_CLEAR:
			status = rcv_cmd_clear();
			break;
		case RCV_HID_OP_ADD: {
			if (args_len < 6) {
				status = RCV_HID_ST_EINVAL;
				break;
			}
			uint64_t addr = 0;
			memcpy(&addr, args, 6);
			status = rcv_cmd_add(addr, &slot);
			break;
		}
		case RCV_HID_OP_REMOVE:
			status = rcv_cmd_remove();
			break;
		case RCV_HID_OP_LIST:
			status = rcv_cmd_list();
			break;
		case RCV_HID_OP_CHANNEL_SET:
			status = rcv_cmd_channel_set((args_len >= 1) ? args[0] : 0);
			break;
		case RCV_HID_OP_CHANNEL_CLEAR:
			status = rcv_cmd_channel_clear();
			break;
		case RCV_HID_OP_RSSI_SCAN:
			status = rcv_cmd_rssi_scan();
			break;
		case RCV_HID_OP_INFO:
			status = rcv_cmd_info();
			break;
		case RCV_HID_OP_UPTIME:
			status = rcv_cmd_uptime();
			break;
		case RCV_HID_OP_STATS: {
			uint32_t dur = 0;
			if (args_len >= 4) {
				dur = sys_get_le32(args);
			}
			status = rcv_cmd_stats(dur);
			break;
		}
		case RCV_HID_OP_RESETSTATS:
			status = rcv_cmd_resetstats();
			break;
		case RCV_HID_OP_COLLECT_START:
			status = rcv_cmd_collect_start((args_len >= 1) ? args[0] : 0);
			break;
		case RCV_HID_OP_COLLECT_STOP:
			status = rcv_cmd_collect_stop();
			break;
		case RCV_HID_OP_REBOOT:
			status = rcv_cmd_reboot();
			break;
		case RCV_HID_OP_DFU:
			status = rcv_cmd_dfu((args_len >= 1) && args[0] != 0);
			break;
		case RCV_HID_OP_TRACKER_CH_ALL:
			status = rcv_cmd_tracker_channel_all((args_len >= 1) ? args[0] : 0);
			if (status == RCV_HID_ST_STARTED) {
				arm_ch_hid_completion(seq, opcode);
			}
			break;
		case RCV_HID_OP_TRACKER_CH_CLR:
			status = rcv_cmd_tracker_channel_clear_all();
			if (status == RCV_HID_ST_STARTED) {
				arm_ch_hid_completion(seq, opcode);
			}
			break;
		default:
			status = RCV_HID_ST_EINVAL;
			break;
		}
	}

	fill_ack(ack_out, seq, opcode, status);
	if (opcode == RCV_HID_OP_ADD && status == RCV_HID_ST_OK && slot >= 0) {
		ack_out[4] = (uint8_t)slot;
	}
	if (opcode == RCV_HID_OP_STATS && status == RCV_HID_ST_OK) {
		ack_out[4] = esb_get_stats_detailed_enabled() ? 1 : 0;
	}
	LOG_INF("HID ACK seq=%u opcode=0x%02X status=%u", seq, opcode, status);
	return true;
}
