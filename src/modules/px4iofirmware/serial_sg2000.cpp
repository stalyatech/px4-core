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

		if (poll_ret <= 0) {
			return -ETIMEDOUT;
		}

		const ssize_t n = read(_fmu_fd, &dst[received], len - received);

		if (n <= 0) {
			return -EIO;
		}

		received += static_cast<size_t>(n);
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
	(void)write(_fmu_fd, &packet, PKT_SIZE(packet));
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

		if (read_full(packet.regs, count * sizeof(uint16_t), 20) != OK) {
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
	(void)tcgetattr(_fmu_fd, &t);
	cfmakeraw(&t);
#ifdef PX4FMU_SERIAL_BAUDRATE
	speed_t baud = B115200;

	switch (PX4FMU_SERIAL_BAUDRATE) {
	case 1500000:
#ifdef B1500000
		baud = B1500000;
#endif
		break;

	case 921600:
#ifdef B921600
		baud = B921600;
#endif
		break;

	default:
		baud = B115200;
		break;
	}

	(void)cfsetspeed(&t, baud);
#endif
	(void)tcsetattr(_fmu_fd, TCSANOW, &t);

	(void)pthread_create(&_rx_thread, nullptr, rx_worker, nullptr);
}
