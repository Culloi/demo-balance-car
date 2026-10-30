#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "uart.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

int uart_open(const char *device)
{
    struct termios settings;
    int fd;
    int saved_errno;

    if (device == NULL) {
        errno = EINVAL;
        return -1;
    }
    /* Keep reads nonblocking even after poll: no race can hang the caller. */
    fd = open(device, O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    if (tcgetattr(fd, &settings) < 0) {
        goto fail;
    }
    settings.c_iflag &= (tcflag_t)~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                                      INLCR | IGNCR | ICRNL | IXON | IXOFF |
                                      IXANY);
    settings.c_oflag &= (tcflag_t)~OPOST;
    settings.c_lflag &= (tcflag_t)~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    settings.c_cflag &= (tcflag_t)~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, B115200) < 0 ||
        cfsetospeed(&settings, B115200) < 0 ||
        tcsetattr(fd, TCSANOW, &settings) < 0) {
        goto fail;
    }
    return fd;

fail:
    saved_errno = errno;
    (void)close(fd);
    errno = saved_errno;
    return -1;
}

int uart_write_all(int fd, const uint8_t *data, size_t len)
{
    size_t sent = 0;

    if (fd < 0 || (len != 0U && data == NULL)) {
        errno = EINVAL;
        return -1;
    }
    while (sent < len) {
        ssize_t count = write(fd, data + sent, len - sent);

        if (count > 0) {
            sent += (size_t)count;
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count == 0) {
            errno = EIO;
            return -1;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            struct pollfd watched = { .fd = fd, .events = POLLOUT,
                                      .revents = 0 };
            int ready;

            do {
                ready = poll(&watched, 1, -1);
            } while (ready < 0 && errno == EINTR);
            if (ready < 0) {
                return -1;
            }
            if ((watched.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                errno = EIO;
                return -1;
            }
            continue;
        }
        return -1;
    }
    return 0;
}

int uart_read_timeout(int fd, uint8_t *data, size_t len, int timeout_ms)
{
    struct timespec now;
    time_t deadline_sec;
    long deadline_nsec;
    size_t received = 0;

    if (fd < 0 || (len != 0U && data == NULL) || len > INT_MAX ||
        timeout_ms < 0) {
        errno = EINVAL;
        return -1;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return -1;
    }
    deadline_sec = now.tv_sec + timeout_ms / 1000;
    deadline_nsec = now.tv_nsec + (long)(timeout_ms % 1000) * 1000000L;
    if (deadline_nsec >= 1000000000L) {
        ++deadline_sec;
        deadline_nsec -= 1000000000L;
    }

    while (received < len) {
        struct pollfd watched = { .fd = fd, .events = POLLIN, .revents = 0 };
        int64_t remaining_ns;
        int remaining_ms;
        int ready;
        ssize_t count;

        if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
            return -1;
        }
        remaining_ns = (int64_t)(deadline_sec - now.tv_sec) * 1000000000LL +
                       deadline_nsec - now.tv_nsec;
        if (remaining_ns <= 0) {
            break;
        }
        remaining_ms = (int)((remaining_ns + 999999LL) / 1000000LL);
        ready = poll(&watched, 1, remaining_ms);
        if (ready < 0 && errno == EINTR) {
            continue;
        }
        if (ready < 0) {
            return -1;
        }
        if (ready == 0) {
            break;
        }
        if ((watched.revents & POLLNVAL) != 0) {
            errno = EBADF;
            return -1;
        }
        if ((watched.revents & POLLIN) == 0) {
            errno = EIO;
            return -1;
        }
        count = read(fd, data + received, len - received);
        if (count > 0) {
            received += (size_t)count;
        } else if (count == 0) {
            errno = EIO;
            return -1;
        } else if (errno != EINTR && errno != EAGAIN &&
                   errno != EWOULDBLOCK) {
            return -1;
        }
    }
    return (int)received;
}

void uart_close(int fd)
{
    if (fd >= 0) {
        (void)close(fd);
    }
}

static int uart_send_target(int fd, int value, int limit, uint8_t cmd,
                            uint8_t sent_frame[7])
{
    uint8_t frame[7];
    uint16_t encoded;

    if (value < -limit || value > limit) {
        errno = ERANGE;
        return -1;
    }
    encoded = (uint16_t)(int16_t)value;
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = cmd;
    frame[3] = 0x02;
    frame[4] = (uint8_t)(encoded & 0xFFU);
    frame[5] = (uint8_t)(encoded >> 8);
    frame[6] = (uint8_t)(frame[2] + frame[3] + frame[4] + frame[5]);

    if (uart_write_all(fd, frame, sizeof(frame)) < 0) {
        return -1;
    }
    if (sent_frame != NULL) {
        (void)memcpy(sent_frame, frame, sizeof(frame));
    }
    return 0;
}

int uart_set_speed(int fd, int rpm, uint8_t frame[7])
{
    return uart_send_target(fd, rpm, 150, 0x02, frame);
}

int uart_set_yaw_rate(int fd, int dps, uint8_t frame[7])
{
    return uart_send_target(fd, dps, 150, 0x03, frame);
}