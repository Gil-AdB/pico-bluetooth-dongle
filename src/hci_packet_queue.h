#ifndef HCI_PACKET_QUEUE_H
#define HCI_PACKET_QUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include "pico.h"

// 64 * 1KB = 64KB per queue (128KB total). Safe for Pico 2.
#define HCI_PACKET_QUEUE_SIZE 64
#define HCI_PACKET_MAX_SIZE 1024

typedef struct __attribute__((aligned(4))) {
    uint8_t packet_type;
    uint16_t size;
    uint8_t data[HCI_PACKET_MAX_SIZE];
} hci_packet_entry_t;

typedef struct {
    uint32_t total;
    uint32_t bytes;
    uint32_t drops;
    uint32_t driver_busy;
    uint32_t peak_depth;
    uint32_t current_depth;
} queue_direction_stats_t;

typedef struct {
    queue_direction_stats_t rx; // Chip -> USB
    queue_direction_stats_t tx; // USB -> Chip
} queue_stats_t;

void hci_packet_queue_init(void);

// --- RX (Upstream: Chip -> USB) ---
bool __not_in_flash_func(hci_rx_enqueue)(uint8_t packet_type, const uint8_t *data, uint16_t size);
hci_packet_entry_t *__not_in_flash_func(hci_rx_peek)(void);
void __not_in_flash_func(hci_rx_free)(void);

// --- TX (Downstream: USB -> Chip) ---
bool __not_in_flash_func(hci_tx_enqueue)(uint8_t packet_type, const uint8_t *data, uint16_t size);
hci_packet_entry_t *__not_in_flash_func(hci_tx_peek)(void);
void __not_in_flash_func(hci_tx_free)(void);

// Diagnostics
void hci_packet_queue_get_stats_and_reset(queue_stats_t *stats_out);

void hci_tx_signal_busy(void);

#endif