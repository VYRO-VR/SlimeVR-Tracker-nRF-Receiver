#include <assert.h>
#include <stdio.h>
#include "rcv_cmd.h"
#include "console_send.h"

static unsigned requests;
static uint8_t last_target, last_flag;
uint8_t rcv_cmd_remote_flag(uint8_t target, uint8_t flag)
{
    assert(rcv_hid_opcode_is_pong_flag(flag));
    requests++;
    last_target = target;
    last_flag = flag;
    return RCV_HID_ST_QUEUED;
}
uint8_t rcv_cmd_tracker_channel_all(uint8_t channel) { assert(false); return 0; }
uint8_t rcv_cmd_tracker_channel_clear_all(void) { assert(false); return 0; }
uint8_t rcv_cmd_remote_sens_set(uint8_t target, float x, float y, float z) { assert(false); return 0; }
uint8_t rcv_cmd_remote_sens_auto(uint8_t target, uint8_t axis, uint16_t rev) { assert(false); return 0; }
uint8_t rcv_cmd_remote_test_on(uint8_t target, uint16_t tps) { assert(false); return 0; }
uint8_t rcv_cmd_remote_test_off(uint8_t target) { assert(false); return 0; }
static uint16_t last_minutes;
uint8_t rcv_cmd_remote_wom_command(uint8_t target, uint16_t minutes)
{
    requests++;
    last_target = target;
    last_minutes = minutes;
    return RCV_HID_ST_STARTED;
}

static void test_wom_commands(void)
{
    char *targets[] = {"0", "all"};
    char *invalid[] = {"", "-1", "+1", "1.5", "1m", "35792", "65534", "65535", "99999999999999999999"};
    for (unsigned i = 0; i < 2; i++) {
        requests = 0;
        console_handle_send(targets[i], "wom", "timeout", NULL, NULL);
        assert(requests == 1 && last_minutes == 0 && last_target == (i ? 255 : 0));
        console_handle_send(targets[i], "wom", "timeout", "1", NULL);
        assert(requests == 2 && last_minutes == 1);
        console_handle_send(targets[i], "wom", "timeout", "35791", NULL);
        assert(requests == 3 && last_minutes == 35791);
        console_handle_send(targets[i], "wom", "timeout", "0", NULL);
        assert(requests == 4 && last_minutes == ESB_WOM_TIMEOUT_CLEAR);
        for (unsigned j = 0; j < sizeof(invalid) / sizeof(invalid[0]); j++) {
            console_handle_send(targets[i], "wom", "timeout", invalid[j], NULL);
        }
        console_handle_send(targets[i], "wom", NULL, NULL, NULL);
        console_handle_send(targets[i], "wom", "off", NULL, NULL);
        console_handle_send(targets[i], "wom", "timeout", "1", "extra");
        assert(requests == 4);
        console_handle_send(targets[i], "wom", "enter", NULL, NULL);
        assert(requests == 5 && last_minutes == ESB_WOM_ENTER && last_target == (i ? 255 : 0));
        console_handle_send(targets[i], "wom", "enter", "0", NULL);
        console_handle_send(targets[i], "wom", "enter", "", NULL);
        console_handle_send(targets[i], "wom", "enter", NULL, "extra");
        console_handle_send(targets[i], "wom", "enter", "1", "extra");
        assert(requests == 5);
    }
}

static void test_calibration_commands(void)
{
    char *targets[] = {"0", "7", "all"};
    for (unsigned i = 0; i < 3; i++) {
        uint8_t target = i == 2 ? RCV_HID_TARGET_ALL : (i == 1 ? 7 : 0);
        requests = 0;
        console_handle_send(targets[i], "calibrate", NULL, NULL, NULL);
        assert(requests == 1 && last_target == target && last_flag == 0x02);
        console_handle_send(targets[i], "calibrate", "acc", NULL, NULL);
        assert(requests == 2 && last_target == target && last_flag == 0x03);
        console_handle_send(targets[i], "6-side", NULL, NULL, NULL);
        assert(requests == 3 && last_target == target && last_flag == 0x03);

        console_handle_send(targets[i], "calibrate", "gyro", NULL, NULL);
        console_handle_send(targets[i], "calibrate", "acc", "extra", NULL);
        console_handle_send(targets[i], "calibrate", "acc", "extra", "more");
        console_handle_send(targets[i], "6-side", "extra", NULL, NULL);
        assert(requests == 3);
    }
    puts("console: calibration aliases select existing wire operations; invalid args never queue");
}

int main(void)
{
    test_calibration_commands();
    test_wom_commands();
    char *targets[] = {"0", "all"};
    for (unsigned i = 0; i < 2; i++) {
        requests = 0;
        console_handle_send(targets[i], "tcal", "heat", NULL, NULL);
        console_handle_send(targets[i], "tcal", "heat", "stop", NULL);
        console_handle_send(targets[i], "tcal", "heat", "status", NULL);
        console_handle_send(targets[i], "tcal", "heat", "44", NULL);
        console_handle_send(targets[i], "tcal", "heat", "start", "44");
        console_handle_send(targets[i], "tcal", "heat", "start", "nan");
        assert(requests == 0);
        console_handle_send(targets[i], "tcal", "heat", "start", NULL);
        assert(requests == 1 && last_target == (i ? RCV_HID_TARGET_ALL : 0));
        assert(last_flag == ESB_PONG_FLAG_TCAL_HEATED_START);
        console_handle_send(targets[i], "tcal", "auto", "on", NULL);
        assert(requests == 2 && last_flag == ESB_PONG_FLAG_TCAL_AUTO_ON);
        console_handle_send(targets[i], "tcal", "off", NULL, NULL);
        assert(requests == 3 && last_flag == ESB_PONG_FLAG_TCAL_OFF);
    }
    puts("console: heated start rejects unsupported controls/targets without queuing");
    return 0;
}
