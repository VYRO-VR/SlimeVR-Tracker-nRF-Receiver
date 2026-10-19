#ifndef SLIMENRF_RECEIVER_CONSOLE_H
#define SLIMENRF_RECEIVER_CONSOLE_H

void console_serial_start(void);
/* DTR close: discard incomplete input, retaining accepted complete commands. */
void console_serial_close(void);
/* USB teardown: also invalidate queued and dequeued commands not yet admitted. */
void console_serial_stop(void);

#endif
