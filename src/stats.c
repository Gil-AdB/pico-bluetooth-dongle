// stats.c - Statistics and LED activity for Pico W Bluetooth Dongle
#include "stats.h"
#include "bsp/board.h"
#include "bt_hci.h"
#include "hardware/timer.h"
#include "hci_packet_queue.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

// --- Profiling Variables ---
static volatile uint32_t prof_c0_loops = 0;
static volatile uint32_t prof_c1_loops = 0;
static volatile uint32_t prof_spi_max_us = 0;
static volatile uint32_t prof_spi_last_us = 0;

// --- TX Gap Timing (Debug) ---
static volatile uint64_t last_tx_time = 0;
static volatile uint32_t tx_gap_max_us = 0;
static volatile uint32_t tx_gap_count = 0;
static volatile uint64_t tx_gap_sum = 0;

// --- LED ---
static bool led_state = false;

void stats_init(void) {
  prof_c0_loops = 0;
  prof_c1_loops = 0;
  prof_spi_max_us = 0;
  last_tx_time = 0;
  tx_gap_max_us = 0;
  tx_gap_count = 0;
  tx_gap_sum = 0;
}

void stats_update_spi_latency(uint32_t us) {
  prof_spi_last_us = us;
  if (us > prof_spi_max_us)
    prof_spi_max_us = us;
}

void stats_increment_core0_loops(void) { prof_c0_loops++; }
void stats_increment_core1_loops(void) { prof_c1_loops++; }

// Debug: Record TX send event for gap timing
void stats_record_tx_send(void) {
  uint64_t now = time_us_64();
  if (last_tx_time > 0) {
    uint32_t gap = (uint32_t)(now - last_tx_time);
    if (gap > tx_gap_max_us)
      tx_gap_max_us = gap;
    tx_gap_sum += gap;
    tx_gap_count++;
  }
  last_tx_time = now;
}

void stats_task(void) {
  static uint32_t last_stats = 0;
  static uint32_t last_led = 0;
  static uint32_t led_tx_snapshot = 0;
  uint32_t now = board_millis();

  // --- LED Activity Indicator ---
  // Blink rate based on TX traffic: 1Hz idle, 2Hz low, 5Hz medium, 10Hz high
  // Uses tx_gap_count which is incremented by stats_record_tx_send()
  uint32_t tx_since_last = tx_gap_count - led_tx_snapshot;
  uint32_t led_interval;
  if (tx_since_last > 50)
    led_interval = 50; // 10Hz - audio streaming
  else if (tx_since_last > 20)
    led_interval = 100; // 5Hz - medium traffic
  else if (tx_since_last > 5)
    led_interval = 250; // 2Hz - low traffic
  else
    led_interval = 500; // 1Hz - idle

  if (now - last_led >= led_interval) {
    last_led = now;
    led_state = !led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
    led_tx_snapshot = tx_gap_count; // Snapshot for next interval
  }

  // --- Stats Printing (every 10s) ---
  if (now - last_stats >= 10000) {
    last_stats = now;

    queue_stats_t s;
    hci_packet_queue_get_stats_and_reset(&s);

    float rx_kbps = (float)s.rx.bytes / 10240.0f;
    float tx_kbps = (float)s.tx.bytes / 10240.0f;

    // Calculate average TX gap
    uint32_t tx_gap_avg =
        (tx_gap_count > 0) ? (uint32_t)(tx_gap_sum / tx_gap_count) : 0;

    printf("\n=== SYSTEM HEALTH (10s) ===\n");
    printf("THROUGHPUT : RX=%.2f KB/s (%lu pkts)  TX=%.2f KB/s (%lu pkts)\n",
           rx_kbps, (unsigned long)s.rx.total, tx_kbps,
           (unsigned long)s.tx.total);
    printf("QUEUES     : RX_Peak=%lu  TX_Peak=%lu  Drops=%lu\n",
           (unsigned long)s.rx.peak_depth, (unsigned long)s.tx.peak_depth,
           (unsigned long)(s.rx.drops + s.tx.drops));
    printf("TX BUSY    : %lu (CYW43 buffer full retries)\n",
           (unsigned long)s.tx.driver_busy);
    printf("CPU LOOP   : Core0=%lu k/s  Core1=%lu k/s\n",
           (unsigned long)(prof_c0_loops / 10000),
           (unsigned long)(prof_c1_loops / 10000));
    printf("SPI LAT    : Max=%lu us  Last=%lu us\n",
           (unsigned long)prof_spi_max_us, (unsigned long)prof_spi_last_us);

    // NEW: TX gap timing (key debug info)
    printf("TX GAP     : Max=%lu us  Avg=%lu us  (>20000 = stutter)\n",
           (unsigned long)tx_gap_max_us, (unsigned long)tx_gap_avg);

    printf("USB ERR    : Reassembly Resets=%lu\n",
           (unsigned long)bt_hci_get_reassembly_errors());
    printf("===========================\n");

    // Reset windowed stats
    prof_c0_loops = 0;
    prof_c1_loops = 0;
    prof_spi_max_us = 0;
    tx_gap_max_us = 0;
    tx_gap_count = 0;
    tx_gap_sum = 0;
  }
}
