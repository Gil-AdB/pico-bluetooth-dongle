// bt_sco.h - SCO (Voice) packet handling for BT dongle
#ifndef BT_SCO_H
#define BT_SCO_H

#include <stdint.h>

// Initialize SCO module
void bt_sco_init(void);

// Set/get ISO alternate setting (0 = inactive, 1-3 = active)
void bt_sco_set_alt_setting(uint8_t alt);
uint8_t bt_sco_get_alt_setting(void);

// Handle incoming SCO packet from CYW43 chip (RX: CYW43 → USB)
void bt_sco_rx_packet(const uint8_t *packet, uint16_t size);

// USB transfer completion callbacks
void bt_sco_tx_complete(void);
void bt_sco_rx_complete(uint8_t *buf, uint16_t len);

// Stats
uint32_t bt_sco_get_rx_count(void);
uint32_t bt_sco_get_tx_count(void);

#endif // BT_SCO_H
