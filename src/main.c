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
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#include <zephyr/dfu/mcuboot.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if defined(CONFIG_BOOTLOADER_MCUBOOT)
static void mcuboot_confirm_thread(void)
{
	k_sleep(K_SECONDS(60));
	if (!boot_is_img_confirmed()) {
		int err = boot_write_img_confirmed();
		if (err) {
			LOG_ERR("Failed to confirm MCUboot image: %d", err);
		} else {
			LOG_INF("MCUboot test image confirmed");
		}
	}
}

K_THREAD_DEFINE(mcuboot_confirm_thread_id, 512, mcuboot_confirm_thread,
		NULL, NULL, NULL, 10, 0, 0);
#endif

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
			if (k_uptime_get() - boot_start > 1000)
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
			if (k_uptime_get() - boot_start > 5000)
			{
				LOG_INF("Boot-time pairing reset requested");
				esb_clear();
				break;
			}
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
