// stats.h - Statistics and LED activity for BT dongle
#ifndef STATS_H
#define STATS_H

#include <stdint.h>

// Initialize stats module
void stats_init(void);

// Call from main loop - handles stats printing and LED
void stats_task(void);

// Profiling access
void stats_update_spi_latency(uint32_t us);
void stats_increment_core0_loops(void);
void stats_increment_core1_loops(void);

// Debug: Record TX send event for gap timing
void stats_record_tx_send(void);

#endif // STATS_H
