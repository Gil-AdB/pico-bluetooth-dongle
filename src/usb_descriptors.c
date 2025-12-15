// usb_descriptors.c - USB descriptors for Pico W Bluetooth Dongle
#include "usb_descriptors.h"
#include <string.h>

// --- Device Descriptor ---
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    // Use TUSB_CLASS_MISC for proper composite device enumeration
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};

// --- Configuration Descriptor ---
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_BTH_DESC_LEN)

uint8_t const desc_configuration[] = {
    // Config descriptor: 2 interfaces (BTH ACL + BTH Voice)
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // BTH Descriptor with isochronous endpoints for SCO
    TUD_BTH_DESCRIPTOR(ITF_NUM_BTH, 0, EPNUM_BT_EVT, 64, 0x01, // Event endpoint
                       EPNUM_BT_ACL_IN, EPNUM_BT_ACL_OUT, 64,  // ACL endpoints
                       EPNUM_BT_ISO_IN, EPNUM_BT_ISO_OUT,
                       9) // ISO endpoints (Alt 1)
};

// --- String Descriptors ---
static char const *string_desc_arr[] = {
    (char[]){0x09, 0x04}, // 0: Language (English)
    "Raspberry Pi",       // 1: Manufacturer
    "Pico W BT Dongle",   // 2: Product
    "123456",             // 3: Serial
};

// --- Descriptor Callbacks ---
uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  static uint16_t _desc_str[32];
  uint8_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
      return NULL;
    const char *str = string_desc_arr[index];
    chr_count = strlen(str);
    if (chr_count > 31)
      chr_count = 31;
    for (uint8_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = str[i];
    }
  }
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
  return _desc_str;
}

// --- BOS Descriptor ---
uint8_t const desc_bos[] = {TUD_BOS_DESCRIPTOR(TUD_BOS_DESC_LEN, 0)};

uint8_t const *tud_descriptor_bos_cb(void) { return desc_bos; }
