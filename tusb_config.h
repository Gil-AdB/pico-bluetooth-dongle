// src/tusb_config.h
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_MCU OPT_MCU_RP2040
#define CFG_TUSB_OS OPT_OS_PICO

// Define the Root Hub Port 0 as a Device port
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED TUSB_SPEED_FULL
#define CFG_TUD_ENDPOINT0_SIZE 64

//--------------------------------------------------------------------
// BTH CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------
// Enable the Bluetooth Host Controller Interface (BTH) class driver
// Enable full debug logging for TinyUSB
// #ifdef CFG_TUSB_DEBUG
// #undef CFG_TUSB_DEBUG
// #endif
// #define CFG_TUSB_DEBUG 2

// Redirect TinyUSB debug output to our custom CDC printf
#define CFG_TUD_BTH                                                            \
  1 // Set Isochronous (audio) endpoint support to 1, as Windows expects it.
#define CFG_TUD_BTH_ISO_ALT_COUNT 2

// #define CFG_TUD_CDC 1
// #define CFG_TUD_CDC_RX_BUFSIZE 512
// #define CFG_TUD_CDC_TX_BUFSIZE 512

#ifdef __cplusplus
}
#endif

#endif
