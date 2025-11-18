// src/tusb_config.h
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

     //--------------------------------------------------------------------
     // COMMON CONFIGURATION
     //--------------------------------------------------------------------
#define CFG_TUSB_MCU                OPT_MCU_RP2040
#define CFG_TUSB_OS                 OPT_OS_PICO

     // Define the Root Hub Port 0 as a Device port
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

#define CFG_TUD_ENABLED             1
#define CFG_TUD_MAX_SPEED           TUSB_SPEED_FULL

     //--------------------------------------------------------------------
     // BTH CLASS DRIVER CONFIGURATION
     //--------------------------------------------------------------------
     // Enable the Bluetooth Host Controller Interface (BTH) class driver
#define CFG_TUD_BTH                 1

     // Set Isochronous (audio) endpoint support to 0, as we are not using it.
#define CFG_TUD_BTH_ISO_ALT_COUNT   0

#ifdef __cplusplus
 }
#endif

#endif
