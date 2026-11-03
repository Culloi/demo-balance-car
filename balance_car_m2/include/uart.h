#ifndef UART_H
#define UART_H

#include <stddef.h>
#include <stdint.h>

/* Open a Linux UART in raw 115200 8N1 mode; return fd or -1 (errno set). */
int uart_open(const char *device);
/* Send all bytes; return 0 on success or -1 (errno set). */
int uart_write_all(int fd, const uint8_t *data, size_t len);
/* Return bytes read (possibly fewer than len on timeout), or -1 on error.
   timeout_ms is a deadline for the entire read, not each byte. */
int uart_read_timeout(int fd, uint8_t *data, size_t len, int timeout_ms);
void uart_close(int fd);

/* Send one motion target. Reject out-of-range values without writing.
   On success, optionally copy the exact seven transmitted bytes to frame.
   Return 0 on success or -1 with errno set on failure. */
int uart_set_speed(int fd, int rpm, uint8_t frame[7]); /* -150..150 RPM */
int uart_set_yaw_rate(int fd, int dps, uint8_t frame[7]); /* -150..150 deg/s */

#endif