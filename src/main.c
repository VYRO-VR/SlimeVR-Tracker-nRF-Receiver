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
#include "system/led.h"
#include "system/status.h"
#include "connection/esb.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	// Initialize LED system first
	set_led(SYS_LED_PATTERN_ACTIVE_PERSIST, SYS_LED_PRIORITY_SYSTEM);

	// Boot sequence with button detection
	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_BOOT);

	uint8_t reboot_counter = reboot_counter_read();
	bool booting_from_shutdown = !reboot_counter;

	// Check if button is pressed during boot
	if (button_read())
	{
		LOG_INF("Button detected during boot");
		int64_t boot_start = k_uptime_get();

		while (button_read())
		{
			int64_t held = k_uptime_get() - boot_start;
			if (held > 5000)
			{
				// Factory reset: the only destructive action, gated behind a
				// deliberate hold-while-plugging-in. Red LED is the commit signal.
				LOG_INF("Boot-time factory reset (clear all pairings)");
				set_led(SYS_LED_PATTERN_ERROR_D, SYS_LED_PRIORITY_HIGHEST);
				esb_clear();
				break;
			}
			if (held > 1000)
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
			k_msleep(10);
		}

		if (k_uptime_get() - boot_start > 5000)
			set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_HIGHEST);
		else if (k_uptime_get() - boot_start > 50) // Short press during boot
			set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_HIGHEST);
	}
	else if (booting_from_shutdown)
	{
		set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_BOOT);
	}

	// Clear boot LED priority
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_BOOT);

	return 0;
}
