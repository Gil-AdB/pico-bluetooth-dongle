// usb_descriptors.h - USB descriptor definitions for BT dongle
#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include "tusb.h"

// USB IDs
#define USB_VID 0x2E8A // Raspberry Pi
#define USB_PID 0x0013 // BT Dongle
#define USB_BCD 0x0200

// Interface numbers
enum { ITF_NUM_BTH = 0, ITF_NUM_BTH_VOICE, ITF_NUM_TOTAL };

// Endpoint addresses
#define EPNUM_BT_EVT 0x81
#define EPNUM_BT_ACL_OUT 0x02
#define EPNUM_BT_ACL_IN 0x82
#define EPNUM_BT_ISO_OUT 0x03
#define EPNUM_BT_ISO_IN 0x83

// Descriptor callbacks (implemented in usb_descriptors.c)
uint8_t const *tud_descriptor_device_cb(void);
uint8_t const *tud_descriptor_configuration_cb(uint8_t index);
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid);
uint8_t const *tud_descriptor_bos_cb(void);

#endif // USB_DESCRIPTORS_H
