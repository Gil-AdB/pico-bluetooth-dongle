#include "log_buffer.h"
#include <string.h>

static char log_buffer[LOG_BUFFER_SIZE];
static uint32_t log_write_pos = 0;
static bool log_wrapped = false;

void log_buffer_init(void) {
  memset(log_buffer, 0, LOG_BUFFER_SIZE);
  log_write_pos = 0;
  log_wrapped = false;
}

void log_buffer_append(const char *str, int len) {
  for (int i = 0; i < len; i++) {
    log_buffer[log_write_pos] = str[i];
    log_write_pos++;
    if (log_write_pos >= LOG_BUFFER_SIZE) {
      log_write_pos = 0;
      log_wrapped = true;
    }
  }
}

void log_buffer_dump(void (*output_fn)(const char *, int)) {
  const char *header = "\n=== LOG BUFFER DUMP ===\n";
  const char *footer = "\n=== END LOG BUFFER ===\n";

  output_fn(header, strlen(header));

  if (log_wrapped) {
    // Print from write_pos to end (oldest data)
    int len1 = LOG_BUFFER_SIZE - log_write_pos;
    if (len1 > 0) {
      output_fn(&log_buffer[log_write_pos], len1);
    }
  }

  // Print from start to write_pos (newest data)
  if (log_write_pos > 0) {
    output_fn(log_buffer, log_write_pos);
  }

  output_fn(footer, strlen(footer));
}

uint32_t log_buffer_get_size(void) {
  if (log_wrapped) {
    return LOG_BUFFER_SIZE;
  }
  return log_write_pos;
}
