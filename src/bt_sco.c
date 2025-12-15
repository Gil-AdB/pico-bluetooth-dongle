// bt_sco.c - SCO (Voice) packet handling for BT dongle
// SCO packets are used for voice calls (HFP/HSP)
// Uses raw TinyUSB endpoint APIs for isochronous transfers

#include "bt_sco.h"
#include "btstack.h"
#include "hci_packet_queue.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "usb_descriptors.h"
#include <device/usbd_pvt.h>
#include <stdio.h>
#include <string.h>

// SCO packet structure: 3-byte header + payload
// Header: Connection Handle (12 bits) + Packet Status Flag (2 bits) + Data
// Length (8 bits)
#define SCO_HEADER_SIZE 3
#define SCO_MAX_PAYLOAD 60
#define SCO_MAX_PACKET (SCO_HEADER_SIZE + SCO_MAX_PAYLOAD)

// HCI transport
static const hci_transport_t *sco_transport = NULL;

// TX buffer (CYW43 -> USB)
static uint8_t sco_tx_buf[SCO_MAX_PACKET];
static volatile bool sco_tx_pending = false;
static uint16_t sco_tx_len = 0;

// RX buffer (USB -> CYW43)
static uint8_t sco_rx_buf[SCO_MAX_PACKET];
static volatile bool sco_rx_ready = false;

// Statistics
static volatile uint32_t sco_rx_count = 0;
static volatile uint32_t sco_tx_count = 0;
static volatile uint32_t sco_tx_errors = 0;

// Current alternate setting (0 = inactive)
static volatile uint8_t current_alt_setting = 0;

void bt_sco_init(void) {
  sco_rx_count = 0;
  sco_tx_count = 0;
  sco_tx_errors = 0;
  sco_tx_pending = false;
  sco_rx_ready = false;
  current_alt_setting = 0;
  sco_transport = hci_transport_cyw43_instance();
  printf("SCO Voice support initialized\n");
}

// Set alternate setting (called from USB stack when host changes alt)
void bt_sco_set_alt_setting(uint8_t alt) {
  current_alt_setting = alt;
  if (alt > 0) {
    printf("[SCO] Alt setting %d activated\n", alt);
    // Queue first RX transfer
    if (!usbd_edpt_busy(0, EPNUM_BT_ISO_OUT)) {
      usbd_edpt_xfer(0, EPNUM_BT_ISO_OUT, sco_rx_buf, SCO_MAX_PACKET);
    }
  } else {
    printf("[SCO] Alt setting 0 (inactive)\n");
  }
}

uint8_t bt_sco_get_alt_setting(void) { return current_alt_setting; }

// Handle incoming SCO packet from CYW43 chip (RX: CYW43 -> USB)
void bt_sco_rx_packet(const uint8_t *packet, uint16_t size) {
  sco_rx_count++;

  // Only forward if voice interface is active
  if (current_alt_setting == 0)
    return;

  // Check if USB endpoint is ready
  if (usbd_edpt_busy(0, EPNUM_BT_ISO_IN)) {
    // Previous transfer not complete, skip
    return;
  }

  // Copy to TX buffer and send
  if (size > SCO_MAX_PACKET)
    size = SCO_MAX_PACKET;
  memcpy(sco_tx_buf, packet, size);
  sco_tx_len = size;
  sco_tx_pending = true;

  if (usbd_edpt_xfer(0, EPNUM_BT_ISO_IN, sco_tx_buf, size)) {
    sco_tx_count++;
  } else {
    sco_tx_errors++;
  }

  // Log occasionally
  if ((sco_rx_count % 500) == 1) {
    printf("[SCO] TX=%lu RX=%lu Err=%lu\n", (unsigned long)sco_tx_count,
           (unsigned long)sco_rx_count, (unsigned long)sco_tx_errors);
  }
}

// Called from USB stack when ISO IN transfer completes
void bt_sco_tx_complete(void) { sco_tx_pending = false; }

// Called from USB stack when ISO OUT transfer completes
void bt_sco_rx_complete(uint8_t *buf, uint16_t len) {
  if (len > 0 && current_alt_setting > 0 && sco_transport) {
    // Forward to CYW43
    sco_transport->send_packet(0x03, buf, len); // HCI_SCO_DATA_PACKET
  }

  // Queue next RX transfer if still active
  if (current_alt_setting > 0) {
    if (!usbd_edpt_busy(0, EPNUM_BT_ISO_OUT)) {
      usbd_edpt_xfer(0, EPNUM_BT_ISO_OUT, sco_rx_buf, SCO_MAX_PACKET);
    }
  }
}

// Get SCO packet counts for stats
uint32_t bt_sco_get_rx_count(void) { return sco_rx_count; }
uint32_t bt_sco_get_tx_count(void) { return sco_tx_count; }
