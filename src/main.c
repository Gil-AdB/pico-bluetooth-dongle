// src/main.c
#include "bsp/board.h"
#include "bth_device.h"
#include "btstack.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"   // Needed for priority settings
#include "hardware/timer.h" // For time_us_64()
#include "hci_packet_queue.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <string.h>

// --- PROFILING VARIABLES ---
static volatile uint32_t prof_c0_loops = 0;
static volatile uint32_t prof_c1_loops = 0;
static volatile uint32_t prof_reassembly_err = 0;
static volatile uint32_t prof_spi_max_us = 0;
static volatile uint32_t prof_spi_last_us = 0;

// --- DEBUGGING ---
// Uncomment to enable verbose serial logs
// #define DEBUG_LOGS

#ifdef DEBUG_LOGS
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

// Buffer for assembling fragmented ACL packets from USB
static uint8_t acl_reassembly_buf[2048];
static uint16_t acl_reassembly_len = 0;

static btstack_packet_callback_registration_t hci_event_callback_registration;

void hci_packet_handler(uint8_t packet_type, uint8_t *packet, uint16_t size);

// Deferred synthetic HCI response for 0x1004
static volatile bool pending_0x1004_response = false;
static volatile uint8_t pending_0x1004_page = 0;

// CRITICAL: Static buffer for synthetic 0x1004 response
// Must be static because TinyUSB transmits asynchronously!
static uint8_t synthetic_response_buffer[16];

#include "pico/multicore.h"

// --- CORE 1: USB MANAGER ---
void __not_in_flash_func(core1_entry)(void) {
  while (1) {
    prof_c1_loops++; // Count iterations
    tud_task();

    // 1. Process RX (Chip -> USB)
    hci_packet_entry_t *rx_pkt = hci_rx_peek();
    if (rx_pkt) {
      bool sent = false;
      while (!sent) {
        if (!tud_mounted()) {
          sent = true;
          break;
        }
        if (rx_pkt->packet_type == HCI_ACL_DATA_PACKET) {
          if (tud_bt_acl_data_send(rx_pkt->data, rx_pkt->size))
            sent = true;
        } else if (rx_pkt->packet_type == HCI_EVENT_PACKET) {
          if (tud_bt_event_send(rx_pkt->data, rx_pkt->size))
            sent = true;
        }
        if (!sent)
          tud_task();
      }
      hci_rx_free();
    }
  }
}

void stats_task(void);

// --- Main Loop ---
int main() {
  // Initialize HCI packet queue FIRST - before ANY other init!
  // This must happen before cyw43_arch_init() which can trigger BT activity
  hci_packet_queue_init();

  // 1. Init System & Queue
  set_sys_clock_khz(240000, true); // OPTIONAL: Overclock to 200MHz
  board_init();

  board_init();
  stdio_init_all();
  printf("Pico W Bluetooth Dongle started.\n");

  // 1. Init with Country Code (Increases TX Power)
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    printf("CYW43 init failed\n");
    return -1;
  }

  // 2. FORCE WIFI OFF (Frees up Air Time)
  cyw43_arch_disable_sta_mode();

  // The CYW43 chip puts the radio to sleep aggressively.
  // This causes Bluetooth Audio packets to be delayed/bunched up.
  // We force "Performance Mode" to keep the radio active.
  // cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1,
  // 1, 1));

  // --- FIX 2: BOOST IRQ PRIORITY ---
  // The CYW43 driver uses DMA/PIO interrupts. We must ensure they are
  // higher priority than anything else (default is 0x80, lower is better).
  // This prevents Core 0 from stalling processing the WiFi/BT chip.
  irq_set_priority(DMA_IRQ_0, 0x40);
  irq_set_priority(DMA_IRQ_1, 0x40);
  irq_set_priority(PIO1_IRQ_0, 0x40); // CYW43 usually on PIO1

  // Ensure USB interrupt is serviced as fast as the WiFi chip.
  // This prevents the USB Hardware Buffer from staying full/busy.
  irq_set_priority(USBCTRL_IRQ, 0x40);

  const hci_transport_t *transport = hci_transport_cyw43_instance();
  transport->init(NULL);
  transport->register_packet_handler(&hci_packet_handler);
  transport->open();

  // Initialize TinyUSB
  tusb_init();

  // 5. Launch USB handling on Core 1
  multicore_launch_core1(core1_entry);

  // Start main loop
  printf("Entering main loop\n");

  // 6. Core 0 Main Loop: Just handles background tasks (LEDs, Wi-Fi/BT IRQs)
  // Note: tud_task() is moved to Core 1. Do NOT call it here.
  while (1) {
    prof_c0_loops++; // Count iterations
    stats_task();

    // Process TX (Downstream)
    hci_packet_entry_t *tx_pkt = hci_tx_peek();
    if (tx_pkt) {
      // MEASURE SPI TIME
      uint64_t start = time_us_64();
      // Try to send to chip
      int result = transport->send_packet(tx_pkt->packet_type, tx_pkt->data,
                                          tx_pkt->size);
      uint32_t dur = (uint32_t)(time_us_64() - start);

      // Update stats
      prof_spi_last_us = dur;
      if (dur > prof_spi_max_us)
        prof_spi_max_us = dur;

      if (result == 0) {
        // SUCCESS: Driver accepted the packet
        hci_tx_free();
      } else {
        // FAILURE: Chip buffer full (or SPI busy)
        // We do NOT free the packet. We will retry it next loop.
        // This creates "Backpressure" on the queue, preventing audio loss.

        // Optional: expose a function to increment this safely
        // For now, if you didn't add the setter function,
        // just rely on audio sounding better.
        hci_tx_signal_busy();

        // Yield briefly to let the driver/IRQ clear the buffer
        busy_wait_us(50);
      }
    }
  }
}

// Handle disconnect complete event in packet handle
void handle_disconnect_event(uint8_t *packet, uint16_t size) {
  if (packet[0] == 0x05 && size >= 4) { // HCI_EVENT_DISCONNECTION_COMPLETE
    // Reset ACL reassembly buffer for this connection
    acl_reassembly_len = 0;
  }
}

// UPSTREAM: CYW43 -> Pico -> Host PC
void __not_in_flash_func(hci_packet_handler)(uint8_t packet_type,
                                             uint8_t *packet, uint16_t size) {
  DBG_PRINTF("[CYW] RX Type=0x%02X Size=%d\n", packet_type, size);

  // Filter BTstack Internal Events (0x60 - 0x6F)
  // Since we are now reading raw data, we might see internal chatter.
  // We filter it so the PC doesn't get confused.
  if (packet_type == HCI_EVENT_PACKET && packet[0] >= 0x60 &&
      packet[0] <= 0x6F) {
    return;
  }

  // // 2. LOAD SHEDDING: Drop LE Advertising Reports if busy
  // // HCI_LE_Meta_Event (0x3E) -> Subevent 0x02 (Advertising Report)
  // // These packets are spammed by nearby devices.
  // if (packet_type == HCI_EVENT_PACKET && packet[0] == 0x3E) {
  //   // Check Subevent Code (Byte 2 of packet)
  //   // Packet: [EventCode 0x3E] [Len] [SubEventCode] ...
  //   if (size > 2 && packet[2] == 0x02) {
  //     uint8_t load = hci_packet_queue_count();
  //     // If queue is > 25% full, start dropping Scan Results
  //     // This prioritizes Audio (ACL Data) over Scanning
  //     if (load > (HCI_PACKET_QUEUE_SIZE / 4)) {
  //       return; // Drop!
  //     }
  //   }
  // }

  // --- IRQ HANDLER (Chip -> RX Queue) ---
  hci_rx_enqueue(packet_type, packet, size);
}

// Called by TinyUSB when an HCI event has been successfully transmitted
void tud_bt_event_sent_cb(uint16_t sent_bytes) {}

// DOWNSTREAM: Host PC -> Pico -> CYW43
void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len) {
  // Safety check: Ensure command has at least opcode (2 bytes)
  if (cmd_len < 2) {
    return;
  }

  uint8_t *cmd = (uint8_t *)hci_cmd;
  uint16_t opcode = cmd[0] | (cmd[1] << 8);
  DBG_PRINTF("[CMD] Opcode=0x%04X Len=%d\n", opcode, cmd_len);

  // Handle HCI Reset (0x0C03) - Reset local state
  if (opcode == 0x0C03) {
    // Reset ACL reassembly buffer
    acl_reassembly_len = 0;
  }

  hci_tx_enqueue(HCI_COMMAND_DATA_PACKET, (uint8_t *)hci_cmd, cmd_len);
}

void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len) {
  // Preventing buffer overflow
  if (acl_reassembly_len + data_len > sizeof(acl_reassembly_buf)) {
    DBG_PRINTF("[ACL] Overflow! Resetting.\n");
    prof_reassembly_err++;
    acl_reassembly_len = 0; // Lost sync
  }

  // Append new data
  memcpy(&acl_reassembly_buf[acl_reassembly_len], acl_data, data_len);
  acl_reassembly_len += data_len;

  DBG_PRINTF("[ACL] RX Chunk=%d Total=%d\n", data_len, acl_reassembly_len);

  // Process Buffer
  while (acl_reassembly_len >= 4) {
    // ACL Header: Handle(2) + DataLen(2)
    uint16_t data_total_len =
        acl_reassembly_buf[2] | (acl_reassembly_buf[3] << 8);
    uint16_t packet_total_len = 4 + data_total_len;

    if (acl_reassembly_len >= packet_total_len) {
      DBG_PRINTF("[ACL] Fwd to CYW43 (Len %d)\n", packet_total_len);
      // We have a complete packet
      hci_tx_enqueue(HCI_ACL_DATA_PACKET, acl_reassembly_buf, packet_total_len);

      // Move remaining data to front
      uint16_t remaining = acl_reassembly_len - packet_total_len;
      if (remaining > 0) {
        memmove(acl_reassembly_buf, &acl_reassembly_buf[packet_total_len],
                remaining);
        acl_reassembly_len = remaining;
      } else {
        acl_reassembly_len = 0;
        break;
      }
    } else {
      // Incomplete packet, wait for more data
      break;
    }
  }
}

// --- USB Descriptors ---
#define USB_VID 0x2E8A // Raspberry Pi
#define USB_PID 0x0013 // A new PID for this project
#define USB_BCD 0x0200

// // --- Device Descriptor ---
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
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 1,                              \
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
  (TUD_CONFIG_DESC_LEN + /*TUD_CDC_DESC_LEN + */ TUD_BTH_DESC_LEN)

uint8_t const desc_configuration[] = {
    // BTH ACL (1) + BTH Voice (1) + CDC (2) = 4 interfaces
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // BTH Descriptor (Standard with ISO)
    // Using 9 bytes for ISO endpoint size (standard for alt setting 0/1
    // variants in some configs)
    TUD_BTH_DESCRIPTOR(ITF_NUM_BTH, 0, EPNUM_BT_EVT, 64, 0x01, EPNUM_BT_ACL_IN,
                       EPNUM_BT_ACL_OUT, 64, EPNUM_BT_ISO_IN, EPNUM_BT_ISO_OUT,
                       9)

    // CDC Descriptor
    // TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
    //                    EPNUM_CDC_IN, 64)
};

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

// --- Statistics & LED Task (Runs on Core 0) ---
void stats_task(void) {
  static uint32_t last_print = 0;
  static uint32_t last_led = 0;
  static bool led_state = false;

  // 1. LED Heartbeat (Every 500ms) - Just to show it hasn't crashed
  if (board_millis() - last_led > 500) {
    last_led = board_millis();
    led_state = !led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
  }

  if (board_millis() - last_print > 10000) {
    last_print = board_millis();
    queue_stats_t s;
    hci_packet_queue_get_stats_and_reset(&s);

    printf("\n=== SYSTEM HEALTH (10s) ===\n");

    // CALCULATE THROUGHPUT (KB/s)
    float rx_kbps = (float)s.rx.bytes / 10240.0f; // Bytes / 10s / 1024
    float tx_kbps = (float)s.tx.bytes / 10240.0f;

    // 1. Throughput
    printf("THROUGHPUT : RX=%.2f KB/s (%lu pkts)  TX=%.2f KB/s (%lu pkts)\n",
           rx_kbps, (unsigned long)s.rx.total, tx_kbps,
           (unsigned long)s.tx.total);
    // 2. Queue Health
    printf("QUEUES   : RX_Peak=%lu  TX_Peak=%lu  Drops=%lu\n",
           (unsigned long)s.rx.peak_depth, (unsigned long)s.tx.peak_depth,
           (unsigned long)(s.rx.drops + s.tx.drops));
    printf("TX BUSY  : %lu (CYW43 buffer full retries)\n",
           (unsigned long)s.tx.driver_busy);

    // 3. CPU Health (The new stuff)
    // A healthy Pico running at 240MHz should loop > 1,000,000 times/sec if
    // idle. If this is < 10,000, something is blocking HARD.
    printf("CPU LOOP : Core0=%lu k/s  Core1=%lu k/s\n",
           (unsigned long)(prof_c0_loops / 10000),
           (unsigned long)(prof_c1_loops / 10000));

    // 4. Latency
    printf("SPI LAT  : Max=%lu us  Last=%lu us\n",
           (unsigned long)prof_spi_max_us, (unsigned long)prof_spi_last_us);

    // 5. Data Integrity
    printf("USB ERR  : Reassembly Resets=%lu\n",
           (unsigned long)prof_reassembly_err);

    printf("===========================\n");

    // Reset windowed profilers
    prof_c0_loops = 0;
    prof_c1_loops = 0;
    prof_spi_max_us = 0;
  }
}

// --- USB Callbacks ---
void tud_mount_cb(void) { printf("USB MOUNTED\n"); }

void tud_umount_cb(void) { printf("USB UNMOUNTED\n"); }

void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  printf("USB SUSPENDED\n");
}

void tud_resume_cb(void) { printf("USB RESUMED\n"); }
