// main.c - Pico W Bluetooth Dongle Entry Point
// Handles system init, Core 0/1 main loops

#include "bsp/board.h"
#include "bt_hci.h"
#include "btstack.h" // For HCI packet types
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hci_packet_queue.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "stats.h"
#include "tusb.h"

// HCI transport handle
static const hci_transport_t *transport;

// --- Core 1: USB Manager ---
void __not_in_flash_func(core1_entry)(void) {
  while (1) {
    stats_increment_core1_loops();
    tud_task();

    // Process RX queue (CYW43 -> USB)
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

// --- USB Callbacks ---
void tud_mount_cb(void) { printf("USB MOUNTED\n"); }
void tud_umount_cb(void) { printf("USB UNMOUNTED\n"); }
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  printf("USB SUSPENDED\n");
}
void tud_resume_cb(void) { printf("USB RESUMED\n"); }

// --- Main ---
int main(void) {
  // 1. Init queue before anything else
  hci_packet_queue_init();
  stats_init();

  // 2. System init
  set_sys_clock_khz(240000, true);
  board_init();
  stdio_init_all();
  printf("Pico W Bluetooth Dongle v2.1 (debug)\n");

  // 3. CYW43 init
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    printf("CYW43 init failed\n");
    return -1;
  }
  cyw43_arch_disable_sta_mode();

  // 4. Boost IRQ priorities for low-latency
  irq_set_priority(DMA_IRQ_0, 0x40);
  irq_set_priority(DMA_IRQ_1, 0x40);
  irq_set_priority(PIO1_IRQ_0, 0x40);
  irq_set_priority(USBCTRL_IRQ, 0x40);

  // 5. Init HCI transport
  transport = hci_transport_cyw43_instance();
  transport->init(NULL);
  transport->register_packet_handler(&hci_packet_handler);
  transport->open();

  // 6. Init USB
  tusb_init();

  // 7. Launch Core 1
  multicore_launch_core1(core1_entry);
  printf("Entering main loop\n");

  // 8. Core 0 loop: TX processing + stats
  while (1) {
    stats_increment_core0_loops();
    stats_task();

    // Process TX queue (USB -> CYW43)
    hci_packet_entry_t *tx_pkt = hci_tx_peek();
    if (tx_pkt) {
      uint64_t start = time_us_64();
      int result = transport->send_packet(tx_pkt->packet_type, tx_pkt->data,
                                          tx_pkt->size);
      uint32_t dur = (uint32_t)(time_us_64() - start);

      stats_update_spi_latency(dur);
      stats_record_tx_send(); // Debug: record TX timing

      if (result == 0) {
        hci_tx_free();
      } else {
        hci_tx_signal_busy();
        busy_wait_us(50);
      }
    }
  }
}
