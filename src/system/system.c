#include "globals.h"
#include "connection/esb.h"
#include "system/led.h"
#include "system/status.h"
#include "retained.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "system.h"

static struct nvs_fs fs;

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

// Button support
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)
#define BUTTON_EXISTS true
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int64_t press_time = 0;
static int64_t last_press_duration = 0;
static void button_thread(void);
K_THREAD_DEFINE(button_thread_id, 1024, button_thread, NULL, NULL, NULL, 6, 0, 0);
#else
#define BUTTON_EXISTS false
#pragma message "Button GPIO does not exist"
#endif

// DFU support check
#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

static bool nvs_init = false;

static int sys_nvs_init(void) {
	if (nvs_init) {
		return 0;
	}
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET;  // starting at NVS_PARTITION_OFFSET
	if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info)) {
		LOG_ERR("Failed to get page info");
		return 1;
	}
	fs.sector_size = info.size;  // sector_size equal to the pagesize
	fs.sector_count = 4U;  // 4 sectors
	int err = nvs_mount(&fs);
	if (err == -EDEADLK) {
		LOG_WRN("All sectors closed, erasing all sectors...");
		err = flash_flatten(
			fs.flash_device,
			fs.offset,
			fs.sector_size * fs.sector_count
		);
		if (!err) {
			err = nvs_mount(&fs);
		}
	}
	if (err) {
		LOG_ERR("Failed to mount NVS");
		return 1;
	}
	nvs_init = true;
	return 0;
}

SYS_INIT(sys_nvs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// TODO: switch back to retained?
uint8_t reboot_counter_read(void) {
	uint8_t reboot_counter;
	nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	return reboot_counter;
}

#if BUTTON_EXISTS
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool pressed = button_read();
	int64_t current_time = k_uptime_get();
	if (press_time && !pressed && current_time - press_time > 50) // debounce
		last_press_duration = current_time - press_time;
	else if (press_time && pressed) // unusual press event on button already pressed
		return;
	press_time = pressed ? current_time : 0;
}

static struct gpio_callback button_cb_data;

static int sys_button_init(void)
{
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
	return 0;
}

SYS_INIT(sys_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

bool button_read(void)
{
#if BUTTON_EXISTS
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

void sys_request_system_off(void)
{
	LOG_INF("System shutdown requested");
	reboot_counter_write(0);
	set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
	k_msleep(2000);
	sys_reboot(SYS_REBOOT_COLD);
}

void sys_request_system_reboot(void)
{
	LOG_INF("System reboot requested");
	sys_reboot(SYS_REBOOT_COLD);
}

#if BUTTON_EXISTS
/*
 * SW0 gestures - one escalating press-and-hold. The LED steps through stages
 * while held, so the user releases on the action they want. Nothing
 * destructive lives here: clearing pairings is boot-only (see main.c) or via
 * the USB console.
 *
 *   tap  (<0.8s)      : exit pairing if active, else a brief "alive" blink
 *   hold 3s   (amber) : shut down all trackers
 *   hold 6s   (blue)  : enter pairing
 *   hold 10s  (red)   : enter DFU / bootloader
 */
#define BTN_TAP_MAX_MS   800
#define BTN_SHUTDOWN_MS  3000
#define BTN_PAIR_MS      6000
#define BTN_DFU_MS       10000

enum btn_stage {
	BTN_STAGE_WINDUP = 0,
	BTN_STAGE_PAIR,
	BTN_STAGE_SHUTDOWN,
	BTN_STAGE_DFU,
};

static void button_enter_dfu(void)
{
#if DFU_EXISTS
#if CONFIG_BUILD_OUTPUT_UF2 // Adafruit bootloader
	NRF_POWER->GPREGRET = 0x57;
#elif CONFIG_BOARD_HAS_NRF5_BOOTLOADER // Nordic Open USB Bootloader
	// BOOTLOADER_DFU_START magic from the Nordic SDK. The bootloader checks
	// GPREGRET on reset and stays in DFU mode if it sees this value.
	NRF_POWER->GPREGRET = 0xB1;
#else
	*dbl_reset_mem = DFU_DBL_RESET_APP;
	ram_range_retain(dbl_reset_mem, sizeof(*dbl_reset_mem), true);
#endif
	k_msleep(100);
#endif
	sys_request_system_reboot();
}

static void button_thread(void)
{
	int64_t press_start_time = 0;
	enum btn_stage stage = BTN_STAGE_WINDUP;
	int64_t last_blink_time = 0;
	bool led_state = false;

	// The button may already be held at boot, where main() treats a long hold
	// as a factory reset. Wait for that press to be released and discard it so
	// the runtime gesture machine never re-acts on the boot hold.
	while (button_read())
		k_msleep(50);
	press_time = 0;
	last_press_duration = 0;

	while (1)
	{
		// Press start - immediate feedback
		if (press_time && !press_start_time)
		{
			press_start_time = press_time;
			stage = BTN_STAGE_WINDUP;
			set_status(SYS_STATUS_BUTTON_PRESSED, true);
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
			last_blink_time = k_uptime_get();
			led_state = true;
			LOG_INF("Button press started");
		}

		// Hold - advance the LED stage by duration so the user can release on
		// the action they want; blink once per second during the wind-up.
		if (press_time && button_read())
		{
			int64_t current_time = k_uptime_get();
			int64_t hold_duration = current_time - press_start_time;

			enum btn_stage new_stage;
			if (hold_duration >= BTN_DFU_MS)
				new_stage = BTN_STAGE_DFU;
			else if (hold_duration >= BTN_PAIR_MS)
				new_stage = BTN_STAGE_PAIR;
			else if (hold_duration >= BTN_SHUTDOWN_MS)
				new_stage = BTN_STAGE_SHUTDOWN;
			else
				new_stage = BTN_STAGE_WINDUP;

			if (new_stage != stage)
			{
				stage = new_stage;
				switch (stage)
				{
				case BTN_STAGE_PAIR: // blue: release to pair
					set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_HIGHEST);
					break;
				case BTN_STAGE_SHUTDOWN: // amber: release to shut down all trackers
					set_led(SYS_LED_PATTERN_LONG_PERSIST, SYS_LED_PRIORITY_HIGHEST);
					break;
				case BTN_STAGE_DFU: // red: release to enter DFU
					set_led(SYS_LED_PATTERN_ERROR_D, SYS_LED_PRIORITY_HIGHEST);
					break;
				default:
					break;
				}
				LOG_INF("Button hold stage %d (%lldms)", stage, hold_duration);
			}

			if (stage == BTN_STAGE_WINDUP && current_time - last_blink_time >= 1000)
			{
				led_state = !led_state;
				set_led(led_state ? SYS_LED_PATTERN_ON : SYS_LED_PATTERN_OFF,
					SYS_LED_PRIORITY_HIGHEST);
				last_blink_time = current_time;
			}
		}

		// Release - act on the stage band the hold landed in
		if (last_press_duration > 50 && press_start_time) // debounce
		{
			int64_t press_duration = last_press_duration;
			last_press_duration = 0;
			press_start_time = 0;
			stage = BTN_STAGE_WINDUP;
			set_status(SYS_STATUS_BUTTON_PRESSED, false);

			if (press_duration < BTN_TAP_MAX_MS)
			{
				// Tap: cancel pairing if open, otherwise just confirm we're alive
				if (get_status(SYS_STATUS_PAIRING_MODE))
				{
					LOG_INF("Tap: exit pairing mode");
					esb_finish_pair();
					set_status(SYS_STATUS_PAIRING_MODE, false);
				}
				else
				{
					LOG_INF("Tap: identify");
				}
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
			}
			else if (press_duration < BTN_SHUTDOWN_MS)
			{
				// Released in the wind-up dead-zone: deliberately do nothing
				LOG_INF("Released before shutdown threshold (%lldms), no action", press_duration);
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			}
			else if (press_duration < BTN_PAIR_MS)
			{
				LOG_INF("Shutdown all trackers");
				esb_request_all_shutdown();
				set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
			}
			else if (press_duration < BTN_DFU_MS)
			{
				LOG_INF("Enter pairing mode");
				set_status(SYS_STATUS_PAIRING_MODE, true);
				esb_start_pairing(); // drives the blue pairing blink (connection priority)
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST); // reveal it
			}
			else
			{
				LOG_INF("DFU mode requested (>=10s)");
				set_led(SYS_LED_PATTERN_ERROR_D, SYS_LED_PRIORITY_HIGHEST);
				button_enter_dfu();
				return; // system will reboot
			}
		}

		k_msleep(50);
	}
}
#endif

void reboot_counter_write(uint8_t reboot_counter) {
	nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
}

// retained not implemented
void sys_write(uint16_t id, void* retained_ptr, const void* data, size_t len) {
	sys_nvs_init();
	int err = nvs_write(&fs, id, data, len);
	if (err < 0)
	{
		LOG_ERR("Failed to write to NVS, error: %d", err);
		return;
	}
}

// reading from nvs
void sys_read(uint16_t id, void* data, size_t len) {
	sys_nvs_init();
	int err = nvs_read(&fs, id, data, len);
	if (err < 0)
	{
		if (err == -ENOENT) // suppress ENOENT
		{
			LOG_DBG("No entry exists for ID %d, read data set to zero", id);
		}
		else
		{
			LOG_ERR("Failed to read from NVS, error: %d", err);
			LOG_WRN("Read data set to zero");
		}
		memset(data, 0, len);
		return;
	}
}
