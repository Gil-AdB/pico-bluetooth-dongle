#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// Defined by compiler flags usually, but force it here
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

// DISABLE DEBUGGING to stop UART spam and timing violations
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif
#define CFG_TUSB_DEBUG 0

// --- Class configuration ---

// CDC
#define CFG_TUD_CDC 1
#define CFG_TUD_CDC_RX_BUFSIZE 1024
#define CFG_TUD_CDC_TX_BUFSIZE 1024

// Bluetooth (BTH)
#define CFG_TUD_BTH 1
// ACL Data Max Size: Large enough for A2DP (3-DH5 ~1021 bytes)
#define CFG_TUD_BTH_RX_BUFSIZE 2048
#define CFG_TUD_BTH_TX_BUFSIZE 2048
// Event/Cmd Max Size: 256 bytes
#define CFG_TUD_BTH_EVENT_BUFSIZE 256

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
