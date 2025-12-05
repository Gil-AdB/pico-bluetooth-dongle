// src/main.c
#include "bsp/board.h"
#include "bth_device.h"
#include "btstack.h"
#include "btstack_memory.h"
#include "hci.h"
#include "pico/btstack_cyw43.h"
#include "pico/btstack_hci_transport_cyw43.h"
// #include "pico/btstack_run_loop_async_context.h" // Removed: File not found
// and unused
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
static volatile int led_state = LED_STATE_IDLE; // Global state
static bool is_dumping_log = false;

// Debug flag - set to true to enable HCI logging (will cause timeouts!)
#define HCI_DEBUG 1

// Deferred synthetic HCI response for 0x1004
static volatile bool pending_0x1004_response = false;
static volatile uint8_t pending_0x1004_page = 0;

// CRITICAL: Static buffer for synthetic 0x1004 response
// Must be static because TinyUSB transmits asynchronously!
static uint8_t synthetic_response_buffer[16];

// --- CDC Logging ---
#include "log_buffer.h"

// Output function for log dump to CDC
void cdc_output(const char *str, int len) {
  if (tud_cdc_connected()) {
    int written = 0;
    while (written < len) {
      int n = tud_cdc_write(str + written, len - written);
      if (n > 0) {
        written += n;
        tud_cdc_write_flush();
      }
      tud_task();        // Let USB stack process
      busy_wait_us(100); // Yield CPU time
    }
  }
}

int cdc_printf(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len > 0) {
    // Store in log buffer
    log_buffer_append(buf, len);

    // Strict Whitelist Filter to prevent infinite recursion
    // We only allow logs that contain specific keywords related to Bluetooth or
    // Control EP. Everything else (especially CDC logs) is blocked.
    bool allowed = false;

    // Whitelist Endpoints: 00 (Control), 81 (Event), 02 (ACL Out), 82 (ACL In)
    if (strstr(buf, "EP 00") || strstr(buf, "EP 0x00") ||
        strstr(buf, "EP 81") || strstr(buf, "EP 0x81") ||
        strstr(buf, "EP 02") || strstr(buf, "EP 0x02") ||
        strstr(buf, "EP 82") || strstr(buf, "EP 0x82")) {
      allowed = true;
    }

    // Whitelist Application Logs
    if (strstr(buf, "HCI") || strstr(buf, "ACL") || strstr(buf, "EV:") ||
        strstr(buf, "CMD:") || strstr(buf, "Synthetic") ||
        strstr(buf, "Pico")) {
      allowed = true;
    }

    if (!allowed) {
      return len;
    }

    // Write to CDC if connected
    cdc_output(buf, len);
  }
  return len;
}

void cdc_task(void) {
  // Re-enable log dumps for debugging
  if (tud_cdc_connected() && tud_cdc_available()) {
    is_dumping_log = true;
    // Flush all pending input first
    char buf[64];
    while (tud_cdc_available()) {
      tud_cdc_read(buf, sizeof(buf));
    }
    // Dump the log buffer once
    log_buffer_dump(cdc_output);
    is_dumping_log = false;
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

  // Initialize log buffer
  log_buffer_init();

  // Initialize the TinyUSB device stack
  tusb_init();

  // Wait for CDC connection (up to 5 seconds) to allow catching startup logs
  uint32_t wait_start = board_millis();
  while (!tud_cdc_connected() && (board_millis() - wait_start < 5000)) {
    tud_task(); // Keep USB stack alive
    sleep_ms(10);
  }

  cdc_printf("Startup complete. Waiting for host...\n");
  tud_task();
  sleep_ms(100);

  // Main firmware loop
  while (1) {
    tud_task();
    cdc_task();
    led_task();

    // Periodic status update (disabled to prevent HCI timeouts)
    // Uncomment for debugging only
    /*
    static uint32_t last_status = 0;
    if (!is_dumping_log && (board_millis() - last_status > 3000)) {
      last_status = board_millis();
      cdc_printf("=== STATUS: USB mounted=%d, BT ready=%d ===\n", tud_mounted(),
                 tud_ready());
    }
    */
  }
}

// UPSTREAM: CYW43 -> Pico -> Host PC
void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet,
                        uint16_t size) {
  switch (packet_type) {
  case 0x04: // HCI Event
  {
    uint8_t event_code = packet[0];

    // Filter out BTstack internal events (event codes >= 0x60)
    if (event_code >= 0x60 && event_code < 0xFF) {
      break; // Silently drop BTstack internal events
    }

#if HCI_DEBUG
    cdc_printf("EV:0x%02x\n", event_code);
#endif

    // Retry sending the event if endpoint is busy
    bool sent = false;
    for (int retry = 0; retry < 10; retry++) {
      if (tud_bt_event_send(packet, size)) {
        sent = true;
        break;
      }
      tud_task();
      busy_wait_us(500);
    }
#if HCI_DEBUG
    if (!sent) {
      cdc_printf("[ERR] EV:0x%02x FAILED\n", event_code);
    }
#endif
  } break;
  case 0x02: // HCI ACL Data (CYW43 -> Host)
  {
#if HCI_DEBUG
    cdc_printf("ACL_UP:%d\n", size);
#endif
    // Retry sending the data if endpoint is busy
    bool acl_sent = false;
    for (int retry = 0; retry < 10; retry++) {
      bool result = tud_bt_acl_data_send(packet, size);
      if (result) {
        acl_sent = true;
        // Process USB stack to allow host to read the data
        for (int i = 0; i < 3; i++) {
          tud_task();
          busy_wait_us(100);
        }
        break;
      }
      // Endpoint busy, process USB stack and retry
      tud_task();
      busy_wait_us(500);
    }
#if HCI_DEBUG
    if (!acl_sent) {
      cdc_printf("[ERR] ACL_UP FAILED %d\n", size);
    }
#endif
  } break;
  default:
    break;
  }
  return;
}

// DOWNSTREAM: Host PC -> Pico -> CYW43
// The original `tud_bt_hci_rx_cb` function is not compatible with TinyUSB's BTH
// device class API. We have implemented `tud_bt_hci_cmd_cb` and
// `tud_bt_acl_data_received_cb` instead. These functions will receive parsed
// HCI commands and ACL data from the USB host.

// Called by TinyUSB when an HCI event has been successfully transmitted
void tud_bt_event_sent_cb(uint16_t sent_bytes) {
#if HCI_DEBUG
  cdc_printf("[USB] Event sent callback: %d bytes transmitted\n", sent_bytes);
#endif
}

// DOWNSTREAM: Host PC -> Pico -> CYW43
void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len) {
  // Safety check: Ensure command has at least opcode (2 bytes)
  if (cmd_len < 2) {
    return;
  }

  uint8_t *cmd = (uint8_t *)hci_cmd;
  uint16_t opcode = cmd[0] | (cmd[1] << 8);

#if HCI_DEBUG
  cdc_printf("CMD:0x%04x\n", opcode);
#endif

  // Intercept HCI Read Local Extended Features (0x1004)
  if (opcode == 0x1004) {
    uint8_t page_number = (cmd_len >= 4) ? cmd[3] : 0;

#if HCI_DEBUG
    cdc_printf("  -> Intercepting 0x1004 (page %d), replying immediately\n",
               page_number);
#endif

    // Prepare Synthetic Response Buffer
    // Must be static to ensure validity after function returns (if passed by
    // reference)
    static uint8_t synth_resp[16];

    synth_resp[0] = 0x0E; // Event Code: Command Complete
    synth_resp[1] = 0x0E; // Parameter Length: 14 bytes
    synth_resp[2] = 0x01; // Num_HCI_Command_Packets: 1
    synth_resp[3] = 0x04; // Opcode low byte
    synth_resp[4] = 0x10; // Opcode high byte (0x1004)

    // Return Parameters
    if (page_number <= 1) {
      synth_resp[5] = 0x00;         // Status: SUCCESS
      synth_resp[6] = page_number;  // Page_Number
      synth_resp[7] = 0x01;         // Max_Page_Number: 1
      memset(&synth_resp[8], 0, 8); // Features: zeros for now
    } else {
      synth_resp[5] = 0x12; // Status: Invalid Parameters
      synth_resp[6] = page_number;
      synth_resp[7] = 0x01;
      memset(&synth_resp[8], 0, 8);
    }

    // Attempt to send response immediately
    // CRITICAL: Do NOT call tud_task() here as we are already in a TinyUSB
    // callback!
    if (tud_bt_event_send(synth_resp, sizeof(synth_resp))) {
      return; // SUCCESS: Sent. DO NOT forward.
    }

// If buffer is full, we have a problem.
// Ideally we should queue this, but for now we log error.
// Forwarding it to CYW43 won't help as it doesn't support it.
#if HCI_DEBUG
    cdc_printf("[ERR] Failed to send synthetic 0x1004 response (BUSY)\n");
#endif
    return;
  }

  // Forward all other commands to CYW43
  hci_transport_cyw43_instance()->send_packet(HCI_COMMAND_DATA_PACKET,
                                              (uint8_t *)hci_cmd, cmd_len);
}

void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len) {
  cdc_printf("TUD_BT_ACL_DATA_RECEIVED_CB: len=%u\n", data_len);
  hci_transport_cyw43_instance()->send_packet(HCI_ACL_DATA_PACKET,
                                              (uint8_t *)acl_data, data_len);
  cdc_printf("  -> Forwarded HCI ACL Data to CYW43.\n");
}

// --- USB Descriptors ---
#define USB_VID 0x2E8A // Raspberry Pi
#define USB_PID 0x0013 // A new PID for this project
#define USB_BCD 0x0200

// --- Device Descriptor ---
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    // Use TUSB_CLASS_MISC for Composite Device (CDC + Bluetooth)
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
// CDC: 2 Interfaces (Comm + Data)
// BT:  2 Interfaces (ACL + SCO)
// Total Interfaces: 4

enum {
  ITF_NUM_BTH = 0,
  ITF_NUM_BTH_VOICE,
  ITF_NUM_CDC,
  ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

// Endpoints
// Endpoints
#define EPNUM_BT_EVT 0x81
#define EPNUM_BT_ACL_OUT 0x02
#define EPNUM_BT_ACL_IN 0x82
#define EPNUM_BT_ISO_OUT 0x03
#define EPNUM_BT_ISO_IN 0x83
#define EPNUM_CDC_NOTIF 0x84
#define EPNUM_CDC_OUT 0x05
#define EPNUM_CDC_IN 0x85

// Custom BTH Descriptor with Empty Alt 0 for ISO
// Length: 8 (IAD) + 30 (ACL) + 9 (ISO Alt 0) + 23 (ISO Alt 1) = 70 bytes
#define TUD_BTH_DESC_LEN_CUSTOM 70
#define TUD_BTH_DESCRIPTOR_CUSTOM(_itfnum, _stridx, _ep_evt, _ep_evt_size,     \
                                  _ep_evt_interval, _ep_in, _ep_out, _ep_size, \
                                  _iso_ep_in, _iso_ep_out, _iso_ep_size)       \
  /* Interface Associate */                                                    \
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2,                              \
      TUSB_CLASS_WIRELESS_CONTROLLER, 0x01,                                    \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, /* Interface 0 (ACL) */           \
      9, TUSB_DESC_INTERFACE, _itfnum, 0, 3, TUSB_CLASS_WIRELESS_CONTROLLER,   \
      0x01, TUD_BT_PROTOCOL_PRIMARY_CONTROLLER,                                \
      _stridx, /* Endpoint In for events */                                    \
      7, TUSB_DESC_ENDPOINT, _ep_evt, TUSB_XFER_INTERRUPT,                     \
      U16_TO_U8S_LE(_ep_evt_size),                                             \
      _ep_evt_interval, /* Endpoint In for ACL data */                         \
      7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size),  \
      1, /* Endpoint Out for ACL data */                                       \
      7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size), \
      1, /* Interface 1 (ISO) Alt 0 - No Endpoints */                          \
      9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 0, 0,                  \
      TUSB_CLASS_WIRELESS_CONTROLLER, 0x01,                                    \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER,                                      \
      0, /* Interface 1 (ISO) Alt 1 - 2 Endpoints */                           \
      9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 1, 2,                  \
      TUSB_CLASS_WIRELESS_CONTROLLER, 0x01,                                    \
      TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, /* Isochronous endpoints */       \
      7, TUSB_DESC_ENDPOINT, _iso_ep_in, TUSB_XFER_ISOCHRONOUS,                \
      U16_TO_U8S_LE(_iso_ep_size), 1, 7, TUSB_DESC_ENDPOINT, _iso_ep_out,      \
      TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_iso_ep_size), 1

#define CONFIG_TOTAL_LEN                                                       \
  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_BTH_DESC_LEN)

uint8_t const desc_configuration[] = {
    // BTH ACL (1) + BTH Voice (1) + CDC (2) = 4 interfaces
    TUD_CONFIG_DESCRIPTOR(1, 4, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // BTH Descriptor (Standard with ISO)
    // Using 9 bytes for ISO endpoint size (standard for alt setting 0/1
    // variants in some configs)
    TUD_BTH_DESCRIPTOR(ITF_NUM_BTH, 0, EPNUM_BT_EVT, 64, 0x01, EPNUM_BT_ACL_IN,
                       EPNUM_BT_ACL_OUT, 64, EPNUM_BT_ISO_IN, EPNUM_BT_ISO_OUT,
                       9),

    // CDC Descriptor
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
                       EPNUM_CDC_IN, 64)};

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
