// bt_hci.h - HCI packet handling for BT dongle
#ifndef BT_HCI_H
#define BT_HCI_H

#include <stddef.h>
#include <stdint.h>

// HCI packet handler for incoming data from CYW43 chip
void hci_packet_handler(uint8_t packet_type, uint8_t *packet, uint16_t size);

// TinyUSB callbacks for HCI commands and ACL data
void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len);
void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len);
void tud_bt_event_sent_cb(uint16_t sent_bytes);

// Get reassembly error count for stats
uint32_t bt_hci_get_reassembly_errors(void);

// Reset HCI state (call on disconnect or HCI reset)
void bt_hci_reset_state(void);

#endif // BT_HCI_H
