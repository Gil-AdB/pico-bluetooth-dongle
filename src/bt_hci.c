// bt_hci.c - HCI packet handling for Pico W Bluetooth Dongle
#include "bt_hci.h"
#include "bt_sco.h"
#include "btstack.h"
#include "hci_packet_queue.h"
#include "pico.h"
#include <string.h>

// --- Debug Logging ---
// #define DEBUG_LOGS
#ifdef DEBUG_LOGS
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) ((void)0)
#endif

// HCI packet types
#define HCI_SCO_DATA_PACKET 0x03

// --- ACL Reassembly ---
static uint8_t acl_reassembly_buf[2048];
static uint16_t acl_reassembly_len = 0;
static uint32_t reassembly_errors = 0;

// --- Public Functions ---

void bt_hci_reset_state(void) { acl_reassembly_len = 0; }

uint32_t bt_hci_get_reassembly_errors(void) { return reassembly_errors; }

// UPSTREAM: CYW43 -> Pico -> Host PC
void __not_in_flash_func(hci_packet_handler)(uint8_t packet_type,
                                             uint8_t *packet, uint16_t size) {
  DBG_PRINTF("[CYW] RX Type=0x%02X Size=%d\n", packet_type, size);

  // Filter BTstack Internal Events (0x60 - 0x6F)
  if (packet_type == HCI_EVENT_PACKET && packet[0] >= 0x60 &&
      packet[0] <= 0x6F) {
    return;
  }

  // SCO packets → SCO handler (voice data)
  if (packet_type == HCI_SCO_DATA_PACKET) {
    bt_sco_rx_packet(packet, size);
    return;
  }

  // Forward to RX queue for Core 1 to send via USB
  hci_rx_enqueue(packet_type, packet, size);
}

// DOWNSTREAM: Host PC -> Pico -> CYW43 (HCI Commands)
void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len) {
  if (cmd_len < 2)
    return;

  uint8_t *cmd = (uint8_t *)hci_cmd;
  uint16_t opcode = cmd[0] | (cmd[1] << 8);
  DBG_PRINTF("[CMD] Opcode=0x%04X Len=%zu\n", opcode, cmd_len);

  // Handle HCI Reset - reset local state
  if (opcode == 0x0C03) {
    bt_hci_reset_state();
  }

  hci_tx_enqueue(HCI_COMMAND_DATA_PACKET, cmd, cmd_len);
}

// DOWNSTREAM: Host PC -> Pico -> CYW43 (ACL Data)
void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len) {
  // Overflow protection
  if (acl_reassembly_len + data_len > sizeof(acl_reassembly_buf)) {
    DBG_PRINTF("[ACL] Overflow! Resetting.\n");
    reassembly_errors++;
    acl_reassembly_len = 0;
    return;
  }

  // Append data
  memcpy(&acl_reassembly_buf[acl_reassembly_len], acl_data, data_len);
  acl_reassembly_len += data_len;

  DBG_PRINTF("[ACL] RX Chunk=%d Total=%d\n", data_len, acl_reassembly_len);

  // Process complete packets
  while (acl_reassembly_len >= 4) {
    uint16_t payload_len = acl_reassembly_buf[2] | (acl_reassembly_buf[3] << 8);
    uint16_t packet_len = 4 + payload_len;

    if (acl_reassembly_len >= packet_len) {
      DBG_PRINTF("[ACL] Fwd to CYW43 (Len %d)\n", packet_len);
      hci_tx_enqueue(HCI_ACL_DATA_PACKET, acl_reassembly_buf, packet_len);

      // Shift remaining data
      uint16_t remaining = acl_reassembly_len - packet_len;
      if (remaining > 0) {
        memmove(acl_reassembly_buf, &acl_reassembly_buf[packet_len], remaining);
      }
      acl_reassembly_len = remaining;
    } else {
      break; // Wait for more data
    }
  }
}

// Called when HCI event sent successfully
void tud_bt_event_sent_cb(uint16_t sent_bytes) { (void)sent_bytes; }
