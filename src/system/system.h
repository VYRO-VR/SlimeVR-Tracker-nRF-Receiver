#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#include "led.h"
#include "status.h"

#define STORED_TRACKERS 1
#define RBT_CNT_ID 2
#define STORED_ADDR_0 3
// 0-15 -> id 3-18
// 0-255 -> id 3-258

#define RF_CHANNEL 259  // NVS key for receiver RF channel

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

void sys_write(uint16_t id, void* ptr, const void* data, size_t len);
void sys_read(uint16_t id, void* data, size_t len);

bool button_read(void);
bool button_read_filtered(void);
void sys_request_system_off(void);
void sys_request_system_reboot(void);

/* Enter bootloader DFU (UF2 default, or OTA when ota=true on Adafruit BL). */
void sys_enter_dfu(bool ota);
/* Mark next reboot to skip Adafruit double-reset DFU entry. */
void sys_skip_dfu_marker(void);

#endif
