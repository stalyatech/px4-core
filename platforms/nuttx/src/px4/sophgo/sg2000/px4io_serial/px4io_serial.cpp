/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 *   Author: Volvox <volvox@stalya.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4io_serial.cpp
 *
 * Serial interface for PX4IO on SG2000
 */

#include <px4_arch/px4io_serial.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <syslog.h>
#include <termios.h>
#include <unistd.h>

#ifndef PX4IO_SERIAL_DEVICE
#error "PX4IO_SERIAL_DEVICE must be defined in board_config.h"
#endif

#ifndef PX4IO_SERIAL_BITRATE
#define PX4IO_SERIAL_BITRATE 1500000
#endif

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][serial] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

uint8_t ArchPX4IOSerial::_io_buffer_storage[sizeof(IOPacket)];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_current_packet(nullptr),
	_completion_semaphore(SEM_INITIALIZER(0))
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{
	if (_fd >= 0) {
		close(_fd);
		_fd = -1;
	}

	px4_sem_destroy(&_completion_semaphore);
}

static speed_t baud_to_speed(unsigned baudrate)
{
	switch (baudrate) {
#ifdef B1500000
	case 1500000: return B1500000;
#endif
#ifdef B921600
	case 921600: return B921600;
#endif
#ifdef B460800
	case 460800: return B460800;
#endif
#ifdef B230400
	case 230400: return B230400;
#endif
	case 115200: return B115200;
	default: return B115200;
	}
}

int ArchPX4IOSerial::init_uart()
{
	PX4IO_DBG("init_uart: open %s @ %u bps\n", PX4IO_SERIAL_DEVICE, PX4IO_SERIAL_BITRATE);
	_fd = open(PX4IO_SERIAL_DEVICE, O_RDWR | O_NOCTTY);

	if (_fd < 0) {
		PX4IO_DBG("init_uart: open failed errno=%d\n", errno);
		return -errno;
	}

	struct termios t{};

	if (tcgetattr(_fd, &t) == 0) {
		cfmakeraw(&t);
		(void)cfsetspeed(&t, baud_to_speed(PX4IO_SERIAL_BITRATE));
		(void)tcsetattr(_fd, TCSANOW, &t);
		PX4IO_DBG("init_uart: termios configured\n");
	}

	px4_sem_init(&_completion_semaphore, 0, 0);
	px4_sem_setprotocol(&_completion_semaphore, SEM_PRIO_NONE);
	PX4IO_DBG("init_uart: ready\n");

	return 0;
}

int
ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	r = init_uart();
	PX4IO_DBG("init: ret=%d\n", r);

	return r;
}

int
ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
{
	switch (operation) {

	case 1:		/* XXX magic number - test operation */
		switch (arg) {
		case 0:
			break;

		case 1: {
				unsigned fails = 0;

				for (unsigned count = 0;; count++) {
					uint16_t value = count & 0xffff;

					if (write((PX4IO_PAGE_TEST << 8) | PX4IO_P_TEST_LED, &value, 1) != 0) {
						fails++;
					}

					if (count >= 5000) {
						syslog(LOG_INFO, "==== test 1 : %u failures ====\n", fails);
						perf_print_counter(_pc_txns);
						perf_print_counter(_pc_retries);
						perf_print_counter(_pc_timeouts);
						perf_print_counter(_pc_crcerrs);
						perf_print_counter(_pc_protoerrs);
						perf_print_counter(_pc_uerrs);
						count = 0;
					}
				}
			}
			break;

		case 2:
			syslog(LOG_INFO, "test 2\n");
			break;

		default:
			break;
		}

	default:
		return -1;
	}

	return 0;
}

static int read_full(int fd, void *buffer, size_t length, int timeout_ms)
{
	uint8_t *dst = reinterpret_cast<uint8_t *>(buffer);
	size_t received = 0;

	while (received < length) {
		struct pollfd pfd{};
		pfd.fd = fd;
		pfd.events = POLLIN;
		const int poll_ret = poll(&pfd, 1, timeout_ms);

		if (poll_ret == 0) {
			return -ETIMEDOUT;
		}

		if (poll_ret < 0) {
			if (errno == EINTR) {
				continue;
			}

			return -errno;
		}

		const ssize_t n = read(fd, &dst[received], length - received);

		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}

			return -errno;
		}

		if (n == 0) {
			return -EIO;
		}

		received += static_cast<size_t>(n);
	}

	return OK;
}

static int write_full(int fd, const void *buffer, size_t length)
{
	const uint8_t *src = reinterpret_cast<const uint8_t *>(buffer);
	size_t written = 0;

	while (written < length) {
		const ssize_t n = write(fd, &src[written], length - written);

		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}

			return -errno;
		}

		if (n == 0) {
			return -EIO;
		}

		written += static_cast<size_t>(n);
	}

	return OK;
}

static void flush_rx(int fd)
{
	uint8_t sink[64];

	while (true) {
		struct pollfd pfd{};
		pfd.fd = fd;
		pfd.events = POLLIN;

		if (poll(&pfd, 1, 0) <= 0) {
			return;
		}

		const ssize_t n = read(fd, sink, sizeof(sink));

		if (n <= 0) {
			return;
		}
	}
}

int
ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	if (_fd < 0) {
		perf_count(_pc_uerrs);
		return -ENODEV;
	}

	_current_packet = _packet;
	perf_begin(_pc_txns);
	flush_rx(_fd);

	const size_t tx_len = PKT_SIZE(*_current_packet);

	if (write_full(_fd, _current_packet, tx_len) != OK) {
		perf_count(_pc_uerrs);
		perf_end(_pc_txns);
		return -EIO;
	}

	if (read_full(_fd, _current_packet, 4, 20) != OK) {
		perf_count(_pc_timeouts);
		perf_end(_pc_txns);
		return -ETIMEDOUT;
	}

	const uint8_t count = PKT_COUNT(*_current_packet);

	if (count > PKT_MAX_REGS) {
		perf_count(_pc_protoerrs);
		perf_end(_pc_txns);
		return -EPROTO;
	}

	if (read_full(_fd, _current_packet->regs, count * sizeof(uint16_t), 20) != OK) {
		perf_count(_pc_timeouts);
		perf_end(_pc_txns);
		return -ETIMEDOUT;
	}

	const uint8_t packet_crc = _current_packet->crc;
	_current_packet->crc = 0;
	const uint8_t computed_crc = crc_packet(_current_packet);
	_current_packet->crc = packet_crc;

	if (packet_crc != computed_crc) {
		perf_count(_pc_crcerrs);
		perf_end(_pc_txns);
		return -EIO;
	}

	if (PKT_CODE(*_current_packet) == PKT_CODE_CORRUPT) {
		perf_count(_pc_protoerrs);
		perf_end(_pc_txns);
		return -EIO;
	}

	perf_end(_pc_txns);
	return OK;
}

int
ArchPX4IOSerial::_interrupt(int irq, void *context, void *arg)
{
	(void)irq;
	(void)context;
	(void)arg;
	return 0;
}

void
ArchPX4IOSerial::_do_interrupt()
{
}
