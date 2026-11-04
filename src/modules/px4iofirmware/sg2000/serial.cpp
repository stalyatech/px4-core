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
#include <sys/ioctl.h>

#include <drivers/drv_hrt.h>

#include "px4io.h"

static int _fmu_fd{-1};
static pthread_t _rx_thread{};

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

/*
 * The PX4IO protocol has no sync byte.  On STM32 every packet is framed by
 * the UART line-idle interrupt; here the bytes come through the NuttX serial
 * upper half, so idle is emulated with timestamps.  The FMU sends a whole
 * packet back to back and then waits up to 10 ms for the reply before it
 * retries, so a pause longer than PX4IO_RX_GAP_US always lies between two
 * packets.  Any bytes held across such a pause belong to a broken packet and
 * are dropped, which brings the parser back in step within one FMU retry.
 */
#define PX4IO_RX_GAP_US		3000
#define PX4IO_RX_POLL_MS	20

struct px4io_link_stats_s {
	uint32_t packets;	/* good packets handled */
	uint32_t replies;	/* replies sent */
	uint32_t crc_errors;	/* complete packets with a bad CRC */
	uint32_t bad_headers;	/* headers with an invalid code or count */
	uint32_t gap_drops;	/* partial packets dropped at a line pause */
	uint32_t stale;		/* packets superseded before a reply was sent */
	uint32_t write_errors;	/* reply writes that failed */
};

/* kept global so it can be read from Linux through /dev/mem */
struct px4io_link_stats_s g_px4io_link_stats;

static void handle_packet(IOPacket &packet)
{
	if (PKT_CODE(packet) == PKT_CODE_WRITE) {
		if (registers_set(packet.page, packet.offset, &packet.regs[0], PKT_COUNT(packet))) {
			packet.count_code = PKT_CODE_ERROR;

		} else {
			packet.count_code = PKT_CODE_SUCCESS;
		}

	} else {
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
	}

	packet.crc = 0;
	packet.crc = crc_packet(&packet);
}

static void send_reply(IOPacket &packet)
{
	if (write_full(&packet, PKT_SIZE(packet)) != OK) {
		g_px4io_link_stats.write_errors++;
		syslog(LOG_ERR, "px4io_sg2000: packet response write failed\n");

	} else {
		g_px4io_link_stats.replies++;
	}
}

static void *rx_worker(void *arg)
{
	(void)arg;
	IOPacket packet{};
	uint8_t *const buf = reinterpret_cast<uint8_t *>(&packet);
	size_t have = 0;
	size_t need = 0;
	bool discard = false;
	hrt_abstime last_rx = 0;
	uint8_t chunk[64];

	while (true) {
		struct pollfd pfd {};
		pfd.fd = _fmu_fd;
		pfd.events = POLLIN;
		const int poll_ret = poll(&pfd, 1, PX4IO_RX_POLL_MS);

		if (poll_ret <= 0) {
			/* line idle: whatever is held is a broken packet */
			if (have > 0) {
				g_px4io_link_stats.gap_drops++;
			}

			have = 0;
			discard = false;
			continue;
		}

		const ssize_t n = read(_fmu_fd, chunk, sizeof(chunk));

		if (n <= 0) {
			continue;
		}

		const hrt_abstime now = hrt_absolute_time();

		if ((last_rx != 0) && (now - last_rx > PX4IO_RX_GAP_US)) {
			if (have > 0) {
				g_px4io_link_stats.gap_drops++;
			}

			have = 0;
			discard = false;
		}

		last_rx = now;

		bool reply_ready = false;

		for (ssize_t i = 0; i < n; i++) {
			if (discard) {
				continue;
			}

			if (reply_ready) {
				/* the FMU sent more after this packet, so it gave up waiting */
				g_px4io_link_stats.stale++;
				reply_ready = false;
			}

			buf[have++] = chunk[i];

			if (have == 4) {
				const uint8_t code = PKT_CODE(packet);

				if ((PKT_COUNT(packet) > PKT_MAX_REGS) ||
				    ((code != PKT_CODE_READ) && (code != PKT_CODE_WRITE))) {
					g_px4io_link_stats.bad_headers++;
					have = 0;
					discard = true;
					continue;
				}

				need = PKT_SIZE(packet);
			}

			if ((have < 4) || (have < need)) {
				continue;
			}

			have = 0;

			const uint8_t crc = packet.crc;
			packet.crc = 0;

			if (crc != crc_packet(&packet)) {
				/* most likely out of step: wait for the line to go quiet */
				g_px4io_link_stats.crc_errors++;
				discard = true;
				continue;
			}

			g_px4io_link_stats.packets++;
			handle_packet(packet);
			reply_ready = true;
		}

		if (reply_ready) {
			int pending = 0;

			if ((ioctl(_fmu_fd, FIONREAD, (unsigned long)&pending) == 0) && (pending > 0)) {
				g_px4io_link_stats.stale++;

			} else {
				send_reply(packet);
			}
		}
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

	struct termios t {};

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
