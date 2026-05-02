/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file serial_sg2000.cpp
 *
 * SG2000 serial transport for PX4IO firmware protocol.
 */

#include <px4_platform_common/px4_config.h>

#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <syslog.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include "px4io.h"

static int _fmu_fd{-1};
static pthread_t _rx_thread{};

static int read_full(void *buffer, size_t len, int timeout_ms)
{
	uint8_t *dst = static_cast<uint8_t *>(buffer);
	size_t received = 0;

	while (received < len) {
		struct pollfd pfd{};
		pfd.fd = _fmu_fd;
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

		const ssize_t n = read(_fmu_fd, &dst[received], len - received);

		if (n <= 0) {
			return -EIO;
		}

		received += static_cast<size_t>(n);
	}

	return OK;
}

static int write_full(const void *buffer, size_t len)
{
	const uint8_t *src = static_cast<const uint8_t *>(buffer);
	size_t written = 0;

	while (written < len) {
		const ssize_t n = write(_fmu_fd, &src[written], len - written);

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

static void handle_packet(IOPacket &packet)
{
	uint8_t crc = packet.crc;
	packet.crc = 0;

	if (crc != crc_packet(&packet)) {
		packet.count_code = PKT_CODE_CORRUPT;
		packet.page = 0xff;
		packet.offset = 0xff;

	} else if (PKT_CODE(packet) == PKT_CODE_WRITE) {
		if (registers_set(packet.page, packet.offset, &packet.regs[0], PKT_COUNT(packet))) {
			packet.count_code = PKT_CODE_ERROR;

		} else {
			packet.count_code = PKT_CODE_SUCCESS;
		}

	} else if (PKT_CODE(packet) == PKT_CODE_READ) {
		unsigned count = 0;
		uint16_t *registers = nullptr;

		if (registers_get(packet.page, packet.offset, &registers, &count) < 0) {
			packet.count_code = PKT_CODE_ERROR;

		} else {
			if (count > PKT_MAX_REGS) {
				count = PKT_MAX_REGS;
			}

			if (count > PKT_COUNT(packet)) {
				count = PKT_COUNT(packet);
			}

			memcpy(&packet.regs[0], registers, count * sizeof(uint16_t));
			packet.count_code = static_cast<uint8_t>(count) | PKT_CODE_SUCCESS;
		}

	} else {
		packet.count_code = PKT_CODE_CORRUPT;
		packet.page = 0xff;
		packet.offset = 0xfe;
	}

	packet.crc = 0;
	packet.crc = crc_packet(&packet);

	if (write_full(&packet, PKT_SIZE(packet)) != OK) {
		syslog(LOG_ERR, "px4io_sg2000: packet response write failed\n");
	}
}

static void *rx_worker(void *arg)
{
	(void)arg;
	IOPacket packet{};

	while (true) {
		if (read_full(&packet, 4, 2000) != OK) {
			continue;
		}

		const uint8_t count = PKT_COUNT(packet);

		if (count > PKT_MAX_REGS) {
			continue;
		}

		/* Bigger inter-byte timeout for the regs portion: a short timeout
		 * + `continue` would leave the regs bytes behind in the RX queue
		 * and the next header read would lock onto garbage, breaking the
		 * stream permanently.  We've already committed to this packet
		 * once the header is in, so wait long enough that scheduler jitter
		 * (cooperative SCHED_RR on the C906 LITTLE) can't trip us.
		 */
		if (read_full(packet.regs, count * sizeof(uint16_t), 200) != OK) {
			continue;
		}

		handle_packet(packet);
	}

	return nullptr;
}

void interface_init(void)
{
#ifndef PX4FMU_SERIAL_DEVICE
#error "PX4FMU_SERIAL_DEVICE must be defined in board_config.h for SG2000 PX4IO firmware"
#endif

	_fmu_fd = open(PX4FMU_SERIAL_DEVICE, O_RDWR | O_NOCTTY);

	if (_fmu_fd < 0) {
		syslog(LOG_ERR, "px4io_sg2000: failed to open %s (%d)\n", PX4FMU_SERIAL_DEVICE, errno);
		return;
	}

	struct termios t{};

	if (tcgetattr(_fmu_fd, &t) < 0) {
		syslog(LOG_ERR, "px4io_sg2000: tcgetattr failed (%d)\n", errno);
		close(_fmu_fd);
		_fmu_fd = -1;
		return;
	}

	cfmakeraw(&t);

	/* SG2000 UART driver has a TCSETS handler that reads termios.c_speed
	 * directly, so any baud (B-macro mask or raw integer) is accepted.
	 * cfsetspeed() looks the value up in g_baud_table — for matched rates
	 * it stores the raw baud in c_speed and the BSD-style mask in CBAUD,
	 * and for non-standard rates it stores the raw baud in c_speed and
	 * sets CBAUD=BOTHER.  Either way the driver consumes c_speed.
	 */
	if (cfsetspeed(&t, PX4FMU_SERIAL_BAUDRATE) < 0) {
		syslog(LOG_ERR, "px4io_sg2000: cfsetspeed(%u) failed (%d)\n",
		       (unsigned)PX4FMU_SERIAL_BAUDRATE, errno);
	}

	if (tcsetattr(_fmu_fd, TCSANOW, &t) < 0) {
		syslog(LOG_ERR, "px4io_sg2000: tcsetattr failed (%d)\n", errno);
		close(_fmu_fd);
		_fmu_fd = -1;
		return;
	}

	/* RX worker must run at highest practical priority so RX bytes are
	 * drained before the upper-half buffer overruns and so the inter-byte
	 * timeout never trips.  The C906 LITTLE core has no preemptive timer
	 * for SCHED_RR slice-end, so equal-priority threads serialise; we
	 * therefore boost ourselves above the default.
	 */
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 4096);

	struct sched_param sp;
	sp.sched_priority = sched_get_priority_max(SCHED_RR) - 1;
	(void)pthread_attr_setschedpolicy(&attr, SCHED_RR);
	(void)pthread_attr_setschedparam(&attr, &sp);
	(void)pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	const int pthread_ret = pthread_create(&_rx_thread, &attr, rx_worker, nullptr);
	pthread_attr_destroy(&attr);

	if (pthread_ret != 0) {
		syslog(LOG_ERR, "px4io_sg2000: failed to start RX thread (%d)\n", pthread_ret);
		close(_fmu_fd);
		_fmu_fd = -1;
	}
}
