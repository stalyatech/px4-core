/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Technology. All rights reserved.
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
 * @file nanoradar_socketcan.hpp
 *
 * Opens a raw SocketCAN socket for the Nanoradar CAN drivers (MR72, NRA15).
 */

#pragma once

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <px4_platform_common/log.h>

namespace nanoradar_can
{

/**
 * Open can<bus>, set the bitrate, install the receive filters and bind.
 *
 * The bitrate can only change while the interface is down, so the interface
 * is taken down, set and brought up again. Equal arbitration and data rates
 * keep the controller on classic CAN.
 *
 * @param bus          0 for can0 (CAN1), 1 for can1 (CAN2)
 * @param filters      receive filters, nullptr to receive every frame
 * @param bitrate_set  true when the bitrate ioctl succeeded
 * @return the socket, or -1 on error
 */
inline int open_socket(int bus, uint32_t bitrate, const struct can_filter *filters, size_t num_filters,
		       bool &bitrate_set)
{
	const char *ifname = (bus == 0) ? "can0" : "can1";
	bitrate_set = false;

	struct ifreq ifr {};
	strlcpy(ifr.ifr_name, ifname, IFNAMSIZ);

	const int ifindex = if_nametoindex(ifname);

	if (ifindex == 0) {
		PX4_ERR("%s: no such interface", ifname);
		return -1;
	}

	const int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (sock < 0) {
		PX4_ERR("%s: socket failed (%d)", ifname, errno);
		return -1;
	}

	ifr.ifr_flags = 0;
	ioctl(sock, SIOCSIFFLAGS, (unsigned long)&ifr);

#if defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL)
	ifr.ifr_ifru.ifru_can_data.arbi_bitrate = bitrate;
	ifr.ifr_ifru.ifru_can_data.arbi_samplep = 0;
	ifr.ifr_ifru.ifru_can_data.data_bitrate = bitrate;
	ifr.ifr_ifru.ifru_can_data.data_samplep = 0;

	if (ioctl(sock, SIOCSCANBITRATE, (unsigned long)&ifr) == 0) {
		bitrate_set = true;

	} else {
		PX4_WARN("%s: setting %u bit/s failed (%d), keeping the default", ifname,
			 static_cast<unsigned>(bitrate), errno);
	}

#else
	PX4_WARN("%s: no CAN bitrate ioctl, the interface must already run at %u bit/s", ifname,
		 static_cast<unsigned>(bitrate));
#endif

	ifr.ifr_flags = IFF_UP;

	if (ioctl(sock, SIOCSIFFLAGS, (unsigned long)&ifr) < 0) {
		PX4_ERR("%s: interface up failed (%d)", ifname, errno);
		close(sock);
		return -1;
	}

	if (filters != nullptr
	    && setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, filters, num_filters * sizeof(struct can_filter)) < 0) {
		PX4_WARN("%s: receive filter failed (%d), reading all frames", ifname, errno);
	}

	struct sockaddr_can addr {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifindex;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("%s: bind failed (%d)", ifname, errno);
		close(sock);
		return -1;
	}

	return sock;
}

/** Match exactly one 11 bit identifier (no extended, no remote frames). */
static constexpr struct can_filter standard_id_filter(uint32_t id)
{
	return {id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
}

} // namespace nanoradar_can
