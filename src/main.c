// src/main.c
#include "bsp/board.h"
#include "bth_device.h"
#include "btstack.h"
#include "btstack_memory.h"
#include "hci.h"
#include "pico/btstack_cyw43.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "pico/btstack_run_loop_async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <stdarg.h>
#include <string.h>

static btstack_packet_callback_registration_t hci_event_callback_registration;

void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet,
                        uint16_t size);
void led_task(void);

// --- LED Status ---
enum {
  LED_STATE_IDLE,     // Blink (1Hz, 700ms ON / 300ms OFF)
  LED_STATE_MOUNTED,  // Fast Blink (5Hz)
  LED_STATE_SUSPENDED // Pulse (Short blink every 2s)
};
static volatile int led_state = LED_STATE_IDLE;

// --- CDC Logging ---
int cdc_printf(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len > 0) {
    // Filter out logs related to the CDC IN endpoint (EP 82) to prevent
    // infinite loops (Logging triggers a transfer, which triggers a log, etc.)
    // Aggressive filter: Ignore anything with "CDC" or the CDC endpoint numbers
    // Also filter "USBD Xfer Complete" because it is printed in a separate call
    // before the endpoint number
    if (strstr(buf, "CDC") != NULL || strstr(buf, "EP 82") != NULL ||
        strstr(buf, "EP 81") != NULL || strstr(buf, "EP 02") != NULL ||
        strstr(buf, "USBD Xfer Complete") != NULL) {
      return len;
    }

    // Write to CDC interface 0
    // Check if connected first to avoid blocking or filling buffer if no host
    if (tud_cdc_connected()) {
      tud_cdc_write(buf, len);
      tud_cdc_write_flush();
    }
  }
  return len;
}

void cdc_task(void) {
  // Connected and there is data available
  if (tud_cdc_connected() && tud_cdc_available()) {
    // Read data to clear buffer (echo back or ignore)
    char buf[64];
    tud_cdc_read(buf, sizeof(buf));
  }
}

// --- Main Loop ---
int main() {
  board_init();
  stdio_init_all();
  printf("Pico W Bluetooth Dongle started.\n");

  if (cyw43_arch_init()) {
    // Initialization failed, blink LED rapidly
    while (true) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      sleep_ms(50);
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
      sleep_ms(50);
    }
  }

  // Initialise BTstack
  if (!btstack_cyw43_init(cyw43_arch_async_context())) {
    printf("Failed to init BTstack CYW43\n");
    // Handle error, e.g., blink LED rapidly
    while (true) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      sleep_ms(50);
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
      sleep_ms(50);
    }
  }

  // inform about BTstack state
  hci_event_callback_registration.callback = &hci_packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);

  // turn on bluetooth!
  hci_power_control(HCI_POWER_ON);

  // Initialize the TinyUSB device stack
  tusb_init();

  // Wait for CDC connection (up to 5 seconds) to allow catching startup logs
  uint32_t wait_start = board_millis();
  while (!tud_cdc_connected() && (board_millis() - wait_start < 5000)) {
    tud_task(); // Keep USB stack alive
    sleep_ms(10);
  }
  printf("Startup complete. Waiting for host...\n");

  // Main firmware loop
  while (1) {
    // Keep the CYW43 and TinyUSB tasks running
    async_context_poll(cyw43_arch_async_context());
    tud_task(); // TinyUSB task scheduler
    cdc_task(); // CDC task (flush/read)
    led_task(); // Update LED status

    // Periodic status print (every 1s)
    static uint32_t last_print = 0;
    if (board_millis() - last_print > 1000) {
      last_print = board_millis();
      printf("Status: Alive. LED State: %d. USB Mounted: %d\n", led_state,
             tud_mounted());
    }
  }
}

// UPSTREAM: CYW43 -> Pico -> Host PC
void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet,
                        uint16_t size) {
  printf("HCI_PACKET_HANDLER: type=0x%02x, channel=0x%04x, size=%u\n",
         packet_type, channel, size);
  switch (packet_type) {
  case 0x04: // HCI Event
    tud_bt_event_send(packet, size);
    printf("  -> Sent HCI Event to USB Host.\n");
    break;
  case 0x02: // HCI ACL Data
    tud_bt_acl_data_send(packet, size);
    printf("  -> Sent HCI ACL Data to USB Host.\n");
    break;
  default:
    printf("  -> Unhandled HCI Packet Type to USB Host: 0x%02x\n", packet_type);
    break;
  }
  return;
}

// DOWNSTREAM: Host PC -> Pico -> CYW43
// The original `tud_bt_hci_rx_cb` function is not compatible with TinyUSB's BTH
// device class API. We have implemented `tud_bt_hci_cmd_cb` and
// `tud_bt_acl_data_received_cb` instead. These functions will receive parsed
// HCI commands and ACL data from the USB host. They will then forward these to
// the BTstack HCI transport.

void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len) {
  printf("TUD_BT_HCI_CMD_CB: len=%u\n", cmd_len);
  hci_transport_cyw43_instance()->send_packet(HCI_COMMAND_DATA_PACKET,
                                              (uint8_t *)hci_cmd, cmd_len);
  printf("  -> Forwarded HCI Command to CYW43.\n");
}

void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len) {
  printf("TUD_BT_ACL_DATA_RECEIVED_CB: len=%u\n", data_len);
  hci_transport_cyw43_instance()->send_packet(HCI_ACL_DATA_PACKET,
                                              (uint8_t *)acl_data, data_len);
  printf("  -> Forwarded HCI ACL Data to CYW43.\n");
}

// --- USB Descriptors ---
#define USB_VID 0x2E8A // Raspberry Pi
#define USB_PID 0x000C // A new PID for this project
#define USB_BCD 0x0200

// --- Device Descriptor ---
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    // Use TUSB_CLASS_MISC for Composite Device (CDC + Bluetooth)
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x01,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};

// --- Configuration Descriptor ---
// CDC: 2 Interfaces (Comm + Data)
// BT:  2 Interfaces (ACL + SCO)
// Total Interfaces: 4

enum {
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_BTH,
  ITF_NUM_BTH_VOICE,
  ITF_NUM_TOTAL
};

// Endpoints
// Endpoints
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define EPNUM_BT_EVT 0x83
#define EPNUM_BT_ACL_OUT 0x04
#define EPNUM_BT_ACL_IN 0x84
#define EPNUM_BT_ISO_OUT 0x05
#define EPNUM_BT_ISO_IN 0x85

// Custom BTH Descriptor with Empty Alt 0 for ISO
// Length: 8 (IAD) + 30 (ACL) + 9 (ISO Alt 0) + 23 (ISO Alt 1) = 70 bytes
#define TUD_BTH_DESC_LEN_CUSTOM 70
#define TUD_BTH_DESCRIPTOR_CUSTOM(_itfnum, _stridx, _ep_evt, _ep_evt_size,     \
                                  _ep_evt_interval, _ep_in, _ep_out, _ep_size, \
                                  _iso_ep_in, _iso_ep_out, _iso_ep_size)       \
  /* Interface Associate */                                                    \
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2,                              \
      TUSB_CLASS_WIRELESS_CONTROLLER, TUSB_SUBCLASS_WIRELESS_RADIO_FREQUENCY,  \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, /* Interface 0 (ACL) */           \
      9, TUSB_DESC_INTERFACE, _itfnum, 0, 3, TUSB_CLASS_WIRELESS_CONTROLLER,   \
      TUSB_SUBCLASS_WIRELESS_RADIO_FREQUENCY,                                  \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER,                                      \
      _stridx, /* Endpoint In for events */                                    \
      7, TUSB_DESC_ENDPOINT, _ep_evt, TUSB_XFER_INTERRUPT,                     \
      U16_TO_U8S_LE(_ep_evt_size),                                             \
      _ep_evt_interval, /* Endpoint In for ACL data */                         \
      7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size),  \
      1, /* Endpoint Out for ACL data */                                       \
      7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size), \
      1, /* Interface 1 (ISO) Alt 0 - No Endpoints */                          \
      9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 0, 0,                  \
      TUSB_CLASS_WIRELESS_CONTROLLER, TUSB_SUBCLASS_WIRELESS_RADIO_FREQUENCY,  \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER,                                      \
      0, /* Interface 1 (ISO) Alt 1 - 2 Endpoints */                           \
      9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 1, 2,                  \
      TUSB_CLASS_WIRELESS_CONTROLLER, TUSB_SUBCLASS_WIRELESS_RADIO_FREQUENCY,  \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, /* Isochronous endpoints */       \
      7, TUSB_DESC_ENDPOINT, _iso_ep_in, TUSB_XFER_ISOCHRONOUS,                \
      U16_TO_U8S_LE(_iso_ep_size), 1, 7, TUSB_DESC_ENDPOINT, _iso_ep_out,      \
      TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_iso_ep_size), 1

#define CONFIG_TOTAL_LEN                                                       \
  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_BTH_DESC_LEN_CUSTOM)

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // CDC Descriptor
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
                       EPNUM_CDC_IN, 64),

    // BTH Descriptor
    TUD_BTH_DESCRIPTOR_CUSTOM(ITF_NUM_BTH, 0, EPNUM_BT_EVT, 16, 0x01,
                              EPNUM_BT_ACL_OUT, EPNUM_BT_ACL_IN, 64,
                              EPNUM_BT_ISO_IN, EPNUM_BT_ISO_OUT, 9)};

char const *string_desc_arr[] = {
    (char[]){0x09, 0x04}, "Raspberry Pi", "Pico W BT Dongle", "123456",
    "CDC Serial" // Index 4
};

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
uint8_t const desc_bos[] = {
    // Total Length, Number of Device Capabilities
    TUD_BOS_DESCRIPTOR(TUD_BOS_DESC_LEN, 0)};

uint8_t const *tud_descriptor_bos_cb(void) { return desc_bos; }

void led_task(void) {
  static uint32_t start_ms = 0;
  static bool led_on = false;
  uint32_t interval = 500;

  switch (led_state) {
  case LED_STATE_IDLE:
    // Asymmetric blink: 900ms ON, 100ms OFF
    if (led_on) {
      if (board_millis() - start_ms > 900) {
        start_ms += 900;
        led_on = false;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
      }
    } else {
      if (board_millis() - start_ms > 100) {
        start_ms += 100;
        led_on = true;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      }
    }
    break;
  case LED_STATE_MOUNTED:
    interval = 100; // 5Hz (100ms on, 100ms off)
    if (board_millis() - start_ms > interval) {
      start_ms += interval;
      led_on = !led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    }
    break;
  case LED_STATE_SUSPENDED:
    // Pulse: ON for 100ms, OFF for 1900ms
    if (led_on) {
      if (board_millis() - start_ms > 100) {
        start_ms += 100;
        led_on = false;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
      }
    } else {
      if (board_millis() - start_ms > 1900) {
        start_ms += 1900;
        led_on = true;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      }
    }
    break;
  }
}

// --- USB Callbacks ---
void tud_mount_cb(void) {
  printf("USB MOUNTED\n");
  led_state = LED_STATE_MOUNTED;
}

void tud_umount_cb(void) {
  printf("USB UNMOUNTED\n");
  led_state = LED_STATE_IDLE;
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  printf("USB SUSPENDED\n");
  led_state = LED_STATE_SUSPENDED;
}

void tud_resume_cb(void) {
  printf("USB RESUMED\n");
  led_state = LED_STATE_MOUNTED;
}
