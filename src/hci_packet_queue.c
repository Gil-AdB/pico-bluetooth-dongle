#include "hci_packet_queue.h"
#include "hardware/sync.h"
#include "pico.h"
#include <string.h>

// --- RX QUEUE (Upstream) ---
static __attribute__((aligned(4))) hci_packet_entry_t rx_q[HCI_PACKET_QUEUE_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile queue_direction_stats_t rx_stats = {0};

// --- TX QUEUE (Downstream) ---
static __attribute__((aligned(4))) hci_packet_entry_t tx_q[HCI_PACKET_QUEUE_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile queue_direction_stats_t tx_stats = {0};

void hci_packet_queue_init(void) {
  rx_head = rx_tail = 0;
  tx_head = tx_tail = 0;
  memset((void*)&rx_stats, 0, sizeof(rx_stats));
  memset((void*)&tx_stats, 0, sizeof(tx_stats));
}

// GENERIC HELPERS (Inline for speed)
static inline bool enqueue(hci_packet_entry_t *q, volatile uint8_t *head, volatile uint8_t tail,
                           volatile queue_direction_stats_t *stats, uint8_t type, const uint8_t *data, uint16_t size) {
    uint8_t next_head = (*head + 1) % HCI_PACKET_QUEUE_SIZE;
    if (next_head == tail) {
        stats->drops++;
        return false;
    }

    // Stats
    stats->total++;
    stats->bytes += size;
    uint8_t depth = (*head >= tail) ? (*head - tail) : ((HCI_PACKET_QUEUE_SIZE - tail) + *head);
    depth++; // Include this one
    if (depth > stats->peak_depth) stats->peak_depth = depth;

    hci_packet_entry_t *entry = &q[*head];
    entry->packet_type = type;
    if (size > HCI_PACKET_MAX_SIZE) size = HCI_PACKET_MAX_SIZE;
    entry->size = size;
    memcpy(entry->data, data, size);

    __dmb();
    *head = next_head;
    return true;
}

static inline hci_packet_entry_t* peek(hci_packet_entry_t *q, volatile uint8_t head, volatile uint8_t tail) {
    if (head == tail) return NULL;
    __dmb();
    return &q[tail];
}

static inline void advance(volatile uint8_t *tail, volatile uint8_t head) {
    if (head == *tail) return;
    __dmb();
    *tail = (*tail + 1) % HCI_PACKET_QUEUE_SIZE;
}

// --- RX IMPLEMENTATION ---
bool __not_in_flash_func(hci_rx_enqueue)(uint8_t type, const uint8_t *data, uint16_t size) {
    return enqueue(rx_q, &rx_head, rx_tail, &rx_stats, type, data, size);
}
hci_packet_entry_t *__not_in_flash_func(hci_rx_peek)(void) {
    return peek(rx_q, rx_head, rx_tail);
}
void __not_in_flash_func(hci_rx_free)(void) {
    advance(&rx_tail, rx_head);
}

// --- TX IMPLEMENTATION ---
bool __not_in_flash_func(hci_tx_enqueue)(uint8_t type, const uint8_t *data, uint16_t size) {
    return enqueue(tx_q, &tx_head, tx_tail, &tx_stats, type, data, size);
}
hci_packet_entry_t *__not_in_flash_func(hci_tx_peek)(void) {
    return peek(tx_q, tx_head, tx_tail);
}
void __not_in_flash_func(hci_tx_free)(void) {
    advance(&tx_tail, tx_head);
}

void hci_tx_signal_busy(void) {
    tx_stats.driver_busy++;
}

// DIAGNOSTICS
void hci_packet_queue_get_stats_and_reset(queue_stats_t *stats_out) {
    uint32_t flags = save_and_disable_interrupts();

    stats_out->rx = (queue_direction_stats_t)rx_stats; // Copy struct
    stats_out->tx = (queue_direction_stats_t)tx_stats;

    // Calc current depths
    stats_out->rx.current_depth = (rx_head >= rx_tail) ? (rx_head - rx_tail) : (HCI_PACKET_QUEUE_SIZE - rx_tail + rx_head);
    stats_out->tx.current_depth = (tx_head >= tx_tail) ? (tx_head - tx_tail) : (HCI_PACKET_QUEUE_SIZE - tx_tail + tx_head);

    // Reset Counters
    rx_stats.total = 0; rx_stats.drops = 0; rx_stats.peak_depth = 0; rx_stats.bytes = 0; rx_stats.driver_busy = 0;
    tx_stats.total = 0; tx_stats.drops = 0; tx_stats.peak_depth = 0; tx_stats.bytes = 0; tx_stats.driver_busy = 0;

    restore_interrupts(flags);
}