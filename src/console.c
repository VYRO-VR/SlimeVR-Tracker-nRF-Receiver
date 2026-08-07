/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "build_defines.h"
#include "parse_args.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/drivers/uart.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log_ctrl.h>
#include "connection/esb.h"
#include "console_send.h"
#include "data_collect.h"
#include "esb_ota.h"
#include "rcv_cmd.h"
#include "rcv_hid_cmd.h"

#include <ctype.h>
#include <stdlib.h>
#include <errno.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
/* below ESB_THREAD_PRIORITY; console must not preempt radio housekeeping */
K_THREAD_DEFINE(console_thread_id, 1024, console_thread, NULL, NULL, NULL, CONSOLE_THREAD_PRIORITY, 0, 0);

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER)

static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};

static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};

static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void print_help(void)
{
	printk(
		"\n=== Available Commands ===\n\n"
		"Device Information:\n"
		"  info                       Get device information\n"
		"  uptime                     Get device uptime\n"
		"  list                       Get paired devices\n"
		"\n"
	);

	printk(
		"Device Management:\n"
		"  reboot                     Soft reset the device\n"
		"  add <address>              Manually add a device\n"
		"  remove                     Remove last device\n"
		"  pair [count]               Enter pairing mode\n"
		"    pair                     Pair indefinitely (timeout after %d seconds)\n"
		"    pair 4                   Exit after pairing 4 new devices\n"
		"  exit                       Exit pairing mode\n"
		"  clear                      Clear stored devices\n"
		"\n",
		CONFIG_PAIRING_TIMEOUT
	);

	printk(
		"Statistics:\n"
		"  stats                      Toggle detailed packet statistics\n"
		"  stats <seconds>            Show detailed stats for N seconds\n"
		"  resetstats                 Reset packet statistics\n"
		"\n"
	);

	printk(
		"RF Channel (Local Receiver):\n"
		"  channel <1-100>            Set receiver RF channel only\n"
		"    Example: channel 25       Set receiver to channel 25\n"
		"  clearchannel               Clear receiver RF channel (use default)\n"
		"\n"
	);

	printk(
		"RSSI / Channel Scan:\n"
		"  rssi_scan                  Scan RSSI across channels 1-100 and print a recommended channel\n"
		"\n"
	);

	printk(
		"Remote Commands:\n"
		"  send <id|all> <command>    Send remote command to tracker(s)\n"
		"    Commands: shutdown, calibrate, 6-side, meow, scan,\n"
		"              mag <on|off|clear|cal|auto on|auto off>, reboot, clear, dfu [ota],\n"
		"              channel <1-100>, clearchannel,\n"
		"              sens <x,y,z|reset|auto <x|y|z> [rev]>,\n"
		"              reset <zro|acc|bat|mag|tcal|fusion>, ping\n"
	);

	printk(
		"    Examples:\n"
		"      send 0 shutdown          Shutdown tracker 0\n"
		"      send all calibrate       Calibrate all active trackers\n"
		"      send 1 meow              Make tracker 1 meow\n"
		"      send 2 reboot            Reboot tracker 2\n"
		"      send 0 sens 1.0,1.0,1.0  Set sensitivity for tracker 0\n"
		"      send 0 sens auto z       Auto-calibrate Z sensitivity on tracker 0\n"
		"      send all sens reset      Reset sensitivity for all\n"
		"      send 1 reset zro         Reset ZRO calibration on tracker 1\n"
		"      send all ping            Ping all active trackers\n"
	);

	printk(
		"      send 3 clear             Clear pairing on tracker 3\n"
		"      send all dfu             Enter UF2 DFU mode on all active trackers\n"
		"      send all dfu ota         Enter OTA DFU mode on all active trackers\n"
		"      send all channel 25      Set all active trackers to channel 25\n"
		"      send all clearchannel    Clear channel for all active trackers\n"
		"\n"
	);

#if DFU_EXISTS
	printk(
		"Bootloader:\n"
		"  dfu [ota]                  Enter DFU bootloader (default UF2, optional OTA)\n"
		"\n"
	);
#endif

	printk(
		"Other:\n"
		"  collect <id>               Start raw sensor data collection from tracker\n"
		"  collect off                Stop data collection\n"
		"  collect                    Show data collection status\n"
		"  ota                        Show ESB OTA update status\n"
		"  ota info <id>              Query firmware info from tracker\n"
		"  ota abort                  Abort active OTA session\n"
		"  meow                       Meow!\n"
		"  help                       Show this help message\n"
		"\n"
	);

	printk(
		"Button Functions:\n"
		"  Short press (1x):          Shutdown all paired trackers (LED double blink)\n"
		"  Quick press (2x):          Exit pairing mode\n"
		"  Quick press (3x):          Enter pairing mode\n"
		"  Long press (5s):           Clear all pairings\n"
	);

#if DFU_EXISTS
	printk("  Long press (10s):          Enter DFU mode\n");
#endif
	printk("\n");
}

static inline void strtolower(char *str)
{
	for (int i = 0; str[i] != '\0'; i++) {
		str[i] = (char)tolower((unsigned char)str[i]);
	}
}

static bool parse_u8_arg(const char *str, uint8_t *value)
{
	char *endptr = NULL;
	unsigned long parsed = strtoul(str, &endptr, 10);

	if (endptr == str || *endptr != '\0' || parsed > UINT8_MAX) {
		return false;
	}

	*value = (uint8_t)parsed;
	return true;
}

static void console_thread(void)
{
#if DFU_EXISTS
	if (button_read()) { // button held on usb connect, enter DFU
		sys_enter_dfu(false);
	}
#endif

	/* Data collection: HID mode uses SYS_INIT, CDC mode needs manual init */
#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
	data_collect_init();
#endif

	console_getline_init();

	// Wait for any pending log data to be processed
	while (log_data_pending()) {
		k_usleep(1);
	}

	// Wait for USB CDC to be ready by checking DTR (Data Terminal Ready) signal
	// This ensures the terminal is actually connected and ready to receive data
	const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (device_is_ready(uart_dev)) {
		uint32_t dtr = 0;
		// Wait up to 5 seconds for DTR to be asserted (terminal connected)
		for (int i = 0; i < 50; i++) {
			if (uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
				break;
			}
			k_msleep(100);
		}
		// Give a bit more time for the terminal to be fully ready
		k_msleep(100);
	}

	printk("*** " CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER " " CONFIG_SLIMEVR_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);

	printk("Type 'help' to show available commands.\n");

	const char command_info[] = "info";
	const char command_uptime[] = "uptime";
	const char command_list[] = "list";
	const char command_reboot[] = "reboot";
	const char command_add[] = "add";
	const char command_remove[] = "remove";
	const char command_pair[] = "pair";
	const char command_exit[] = "exit";
	const char command_clear[] = "clear";
	const char command_stats[] = "stats";
	const char command_resetstats[] = "resetstats";
	const char command_channel[] = "channel";
	const char command_clearchannel[] = "clearchannel";
	const char command_rssi_scan[] = "rssi_scan";
	const char command_send[] = "send";
	const char command_help[] = "help";

#if DFU_EXISTS
	const char command_dfu[] = "dfu";
#endif

	const char command_meow[] = "meow";
	const char command_collect[] = "collect";
	const char command_ota[] = "ota";

	while (1) {
		char *line = console_getline();
		char *argv[8] = {NULL};
		size_t argc = parse_args(line, argv, ARRAY_SIZE(argv));
		if (argc == 0) {
			continue;
		}
		for (size_t i = 0; i < argc; i++) {
			strtolower(argv[i]);
		}

		char *arg = argc > 1 ? argv[1] : NULL;
		char *arg2 = argc > 2 ? argv[2] : NULL;
		char *arg3 = argc > 3 ? argv[3] : NULL;
		char *arg4 = argc > 4 ? argv[4] : NULL;
		char *arg5 = argc > 5 ? argv[5] : NULL;

		if (strcmp(argv[0], command_help) == 0) {
			print_help();
		} else if (strcmp(argv[0], command_info) == 0) {
			rcv_cmd_info();
		} else if (strcmp(argv[0], command_uptime) == 0) {
			rcv_cmd_uptime();
		} else if (strcmp(argv[0], command_add) == 0) {
			if (argc != 2) {
				printk("Invalid number of arguments\n");
				continue;
			}
			uint64_t addr = parse_u64(arg, 16);
			char buf[13];
			snprintk(buf, 13, "%012llx", addr);
			if (addr != 0 && strcmp(buf, arg) == 0) {
				int8_t slot = -1;
				uint8_t st = rcv_cmd_add(addr, &slot);
				if (st == RCV_HID_ST_OK) {
					printk("Tracker stored in slot %d\n", slot);
				} else if (st == RCV_HID_ST_ENOSPC) {
					printk("Tracker list is full\n");
				} else {
					printk("Invalid tracker address\n");
				}
			} else {
				printk("Invalid address\n");
			}
		} else if (strcmp(argv[0], command_remove) == 0) {
			rcv_cmd_remove();
		} else if (strcmp(argv[0], command_list) == 0) {
			rcv_cmd_list();
		} else if (strcmp(argv[0], command_reboot) == 0) {
			rcv_cmd_reboot();
		} else if (strcmp(argv[0], command_pair) == 0) {
			if (!arg) {
				rcv_cmd_pair(0);
				printk("Pairing mode enabled (auto-exit after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
			} else {
				char *endptr;
				long count = strtol(arg, &endptr, 10);
				if (*endptr != '\0' || count < 0 || count > 255) {
					printk("Invalid count. Usage: pair [count]\n");
					printk("  pair       - Pair indefinitely (timeout after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
					printk("  pair 4     - Exit after pairing 4 new devices\n");
				} else if (count == 0) {
					rcv_cmd_pair(0);
					printk("Pairing mode enabled (auto-exit after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
				} else {
					rcv_cmd_pair((uint8_t)count);
					printk(
						"Pairing mode enabled (auto-exit after %u new devices or %d seconds)\n",
						(uint8_t)count,
						CONFIG_PAIRING_TIMEOUT
					);
				}
			}
		} else if (strcmp(argv[0], command_exit) == 0) {
			rcv_cmd_exit_pair();
		} else if (strcmp(argv[0], command_clear) == 0) {
			rcv_cmd_clear();
		} else if (strcmp(argv[0], command_stats) == 0) {
			if (!arg) {
				rcv_cmd_stats(0);
				if (esb_get_stats_detailed_enabled()) {
					printk("Detailed stats enabled (toggle again to disable)\n");
				} else {
					printk("Detailed stats disabled\n");
				}
			} else {
				char *endptr;
				long duration = strtol(arg, &endptr, 10);
				if (*endptr != '\0' || duration < 0 || duration > 86400) {
					printk("Invalid duration. Usage: stats [seconds]\n");
					printk("  stats       - Toggle detailed stats on/off\n");
					printk("  stats 30    - Show detailed stats for 30 seconds\n");
				} else if (duration == 0) {
					rcv_cmd_stats(0);
					if (esb_get_stats_detailed_enabled()) {
						printk("Detailed stats enabled (toggle again to disable)\n");
					} else {
						printk("Detailed stats disabled\n");
					}
				} else {
					rcv_cmd_stats((uint32_t)duration);
					printk("Detailed stats enabled for %ld seconds\n", duration);
				}
			}
		} else if (strcmp(argv[0], command_resetstats) == 0) {
			rcv_cmd_resetstats();
		} else if (strcmp(argv[0], command_rssi_scan) == 0) {
			/* Same busy gate as HID; avoids concurrent esb_deinitialize. */
			uint8_t st = rcv_cmd_rssi_scan();
			if (st == RCV_HID_ST_EBUSY) {
				printk("RSSI scan already in progress\n");
			} else if (st == RCV_HID_ST_STARTED) {
				printk("RSSI scan started\n");
			}
		} else if (strcmp(argv[0], command_channel) == 0) {
			if (!arg) {
				printk("Usage: channel <1-100>\n");
				printk("Example: channel 25 - Set receiver RF channel to 25 (local only)\n");
			} else {
				char *endptr;
				long channel = strtol(arg, &endptr, 10);

				if (*endptr != '\0' || channel < 1 || channel > 100) {
					printk("Invalid channel. Must be a number between 1 and 100.\n");
				} else if (rcv_cmd_channel_set((uint8_t)channel) == RCV_HID_ST_OK) {
					printk("Receiver RF channel set to %d (local only)\n", (int)channel);
				}
			}
		} else if (strcmp(argv[0], command_clearchannel) == 0) {
			rcv_cmd_channel_clear();
			printk("Receiver RF channel cleared (local only)\n");
		} else if (strcmp(argv[0], command_send) == 0) {
			console_handle_send(arg, arg2, arg3, arg4, arg5);
		}
#if DFU_EXISTS
		else if (strcmp(argv[0], command_dfu) == 0) {
			bool ota = false;
			if (arg) {
				if (strcmp(arg, "ota") == 0) {
					ota = true;
				} else {
					printk("Unknown dfu argument: %s (use 'ota' or omit it)\n", arg);
					continue;
				}
			}
			if (rcv_cmd_dfu(ota) == RCV_HID_ST_ENOTSUP) {
				printk("DFU not available on this build\n");
			}
		}
#endif
		else if (strcmp(argv[0], command_meow) == 0) {
			print_meow();
		} else if (strcmp(argv[0], command_collect) == 0) {
#ifdef CONFIG_DATA_COLLECT
			if (arg && strcmp(arg, "off") == 0) {
				if (data_collect_is_active()) {
					uint8_t tid = data_collect_get_target_id();
					rcv_cmd_collect_stop();
					printk("Data collection stopped, sent OFF to tracker %u\n", tid);
				} else {
					printk("Data collection is not active\n");
				}
			} else if (arg) {
				char *endptr = NULL;
				unsigned long id = strtoul(arg, &endptr, 10);
				if (endptr != arg && *endptr == '\0' && id < 255) {
					if (rcv_cmd_collect_start((uint8_t)id) == RCV_HID_ST_OK) {
						printk("Data collection started for tracker %u\n", (unsigned)id);
						printk("Test mode enabled on tracker (prevents sleep)\n");
						printk("Non-target trackers will receive SHUTDOWN\n");
						printk("Use 'collect off' to stop\n");
					}
				} else {
					printk("Invalid tracker ID: %s\n", arg);
				}
			} else {
				if (data_collect_is_active()) {
					printk("Data collection ACTIVE for tracker %u\n", data_collect_get_target_id());
				} else {
					printk("Data collection inactive\n");
					printk("Usage: collect <tracker_id> | collect off\n");
				}
			}
#else
			printk("Data collection not available (build with CONFIG_DATA_COLLECT=y)\n");
#endif
		} else if (strcmp(argv[0], command_ota) == 0) {
			if (!arg) {
				/* "ota" with no args → show status */
				esb_ota_relay_console_cmd(0, "status");
			} else if (strcmp(arg, "abort") == 0 || strcmp(arg, "cancel") == 0) {
				if (arg2) {
					uint8_t id;
					if (!parse_u8_arg(arg2, &id)) {
						printk("Invalid tracker ID: %s\n", arg2);
						continue;
					}
					esb_ota_relay_console_cmd(id, "abort");
				} else {
					/* Abort all OTA targets */
					esb_ota_relay_console_cmd(0xFF, "abort");
				}
			} else if (strcmp(arg, "status") == 0) {
				esb_ota_relay_console_cmd(0, "status");
			} else if (strcmp(arg, "info") == 0) {
				if (arg2) {
					uint8_t id;
					if (!parse_u8_arg(arg2, &id)) {
						printk("Invalid tracker ID: %s\n", arg2);
						continue;
					}
					esb_ota_relay_console_cmd(id, "info");
				} else {
					printk("Usage: ota info <tracker_id>\n");
				}
			} else {
				printk("OTA commands: ota, ota info <id>, ota abort, ota status\n");
			}
		} else {
			printk("Unknown command\n");
		}
	}
}

#endif
