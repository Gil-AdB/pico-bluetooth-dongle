#include "src/log_buffer.h"
#include <stdio.h>
#include <string.h>

// Output function for testing
void test_output(const char *str, int len) {
  fwrite(str, 1, len, stdout);
  fflush(stdout);
}

int main() {
  printf("Testing log buffer...\n\n");

  log_buffer_init();

  // Test 1: Simple append
  printf("Test 1: Simple append\n");
  log_buffer_append("Line 1\n", 7);
  log_buffer_append("Line 2\n", 7);
  log_buffer_append("Line 3\n", 7);
  printf("Buffer size: %u\n", log_buffer_get_size());
  log_buffer_dump(test_output);
  printf("\n");

  // Test 2: Wrap around
  printf("Test 2: Fill buffer to test wrap\n");
  log_buffer_init();
  for (int i = 0; i < 100; i++) {
    char buf[64];
    int len =
        snprintf(buf, sizeof(buf),
                 "Message #%d - Testing circular buffer wrap around\n", i);
    log_buffer_append(buf, len);
  }
  printf("Buffer size after 100 messages: %u (should be %d)\n",
         log_buffer_get_size(), LOG_BUFFER_SIZE);
  log_buffer_dump(test_output);
  printf("\n");

  // Test 3: Append after wrap
  printf("Test 3: Continue appending after wrap\n");
  log_buffer_append("FINAL MESSAGE AFTER WRAP\n", 26);
  log_buffer_dump(test_output);

  printf("\nAll tests complete!\n");
  return 0;
}
