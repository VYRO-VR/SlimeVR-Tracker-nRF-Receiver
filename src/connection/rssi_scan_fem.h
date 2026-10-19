#ifndef RSSI_SCAN_FEM_H
#define RSSI_SCAN_FEM_H
#include <stdbool.h>

/* Exclusive raw owner only: ESB must be fully deinitialized and RADIO Disabled.
 * Prepare settles the normal MPSL-configured RX frontend before RXEN. Release
 * after every visit (also failures), before ESB initialization. A release error
 * is unsafe cleanup: retain ownership and do not initialize ESB.
 */
#if defined(CONFIG_MPSL_FEM)
int rssi_scan_fem_prepare(void);
int rssi_scan_fem_release(void);
bool rssi_scan_fem_is_safe(void);
#else
static inline int rssi_scan_fem_prepare(void) { return 0; }
static inline int rssi_scan_fem_release(void) { return 0; }
static inline bool rssi_scan_fem_is_safe(void) { return true; }
#endif

#endif
