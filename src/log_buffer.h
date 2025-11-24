#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_BUFFER_SIZE 8192

void log_buffer_init(void);
void log_buffer_append(const char *str, int len);
void log_buffer_dump(void (*output_fn)(const char *, int));
void log_buffer_clear(void); // Clear buffer contents
uint32_t log_buffer_get_size(void);

#endif // LOG_BUFFER_H
