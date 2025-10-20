/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file fdcan.c
 *
 * Board-specific CAN functions.
 */

#if !defined(CONFIG_CAN)

#include <stdint.h>

#include "board_config.h"


__EXPORT
uint16_t board_get_can_interfaces(void)
{
	uint16_t enabled_interfaces = 0x3;

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_CAN2)) {
		enabled_interfaces &= ~(1 << 1);
	}

	return enabled_interfaces;
}

#else

#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/can.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <net/if.h>
#include <netpacket/can.h>

#include "chip.h"
#include "stm32_fdcan_sock.h"
#include "board_config.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * Name: socketcan_test
 *
 ****************************************************************************/

int socketcan_test(int bus)
{
	struct can_frame frame_config;
	struct sockaddr_can addr;
	struct ifreq ifr;
	int sock;
	int ret = 0;

	/* Select interface and pins */

	switch (bus)
	{
	case 0:
		strlcpy(ifr.ifr_name, "can0", IFNAMSIZ);
		break;

	case 1:
		strlcpy(ifr.ifr_name, "can1", IFNAMSIZ);
		break;
	}

	/* Find network interface */

	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex)
	{
		canerr("CAN%d : if_nametoindex failed\n", bus);
		return -1;
	}

	/* Configure pins */

	/* Init CAN frames, e.g. LEN = 0 */

	memset(&frame_config, 0, sizeof(frame_config));

	/* Prepare CAN frames. Refer to the TJA1153 datasheets and application
	* hints available on NXP.com for details.
	*/

	frame_config.can_id  = 0x18da00f1 | CAN_EFF_FLAG;
	frame_config.can_dlc = 6;
	frame_config.data[0] = 0x10;
	frame_config.data[1] = 0x00;
	frame_config.data[2] = 0x50;
	frame_config.data[3] = 0x00;
	frame_config.data[4] = 0x07;
	frame_config.data[5] = 0xff;

	/* Open socket */

	if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		canerr("CAN%d : Failed to open socket\n", bus);
		return -1;
	}

	/* Bring up the interface */

	ifr.ifr_flags = IFF_UP;
	ret = ioctl(sock, SIOCSIFFLAGS, (unsigned long)&ifr);
	if (ret < 0)
	{
		canerr("CAN%d : ioctl failed (can't set interface flags)\n", bus);
		close(sock);
		return -1;
	}

	/* Initialize sockaddr struct */

	memset(&addr, 0, sizeof(addr));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* Disable default receive filter on this RAW socket
	*
	* This is obsolete as we do not read from the socket at all, but for this
	* reason we can remove the receive list in the kernel to save a little
	* (really very little!) CPU usage.
	*/

	setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* Bind socket and send the CAN frames */

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		canerr("CAN%d : Failed to bind socket\n", bus);
		close(sock);
		return -1;
	}

	if (write(sock, &frame_config, CAN_MTU) != CAN_MTU)
	{
		canerr("CAN%d : Failed to write frame_config\n", bus);
		close(sock);
		return -1;
	}

	/* Sleep for 100 ms to ensure that CAN frames have been transmitted */

	nxsig_usleep(100 * 1000);

	/* TJA1153 must be taken out of STB mode */

	/* Bring down the interface */

	ifr.ifr_flags = IFF_DOWN;
	ret = ioctl(sock, SIOCSIFFLAGS, (unsigned long)&ifr);
	if (ret < 0)
	{
		canerr("CAN%d : ioctl failed (can't set interface flags)\n", bus);
		close(sock);
		return -1;
	}

	close(sock);
	caninfo("CAN%d configuration successful\n", bus);
	return 0;
}


/************************************************************************************
 * Public Functions
 ************************************************************************************/
int can_devinit(void);

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
	static int initialized = 0;
	int ret;

	/* Check if we have already initialized */
	if (initialized == 0) {

		#ifdef CONFIG_STM32H7_FDCAN1

		/* Call stm32_fdcansockinitialize() to get an instance of the FDCAN interface */

		ret = stm32_fdcansockinitialize(0);
		if (ret < 0)
			{
			canerr("ERROR:  Failed to get FDCAN interface %d\n", ret);
			return ret;
			}

		/* test the socket */
		socketcan_test(0);

		/* Now we are initialized */
		initialized++;
		#endif

		#ifdef CONFIG_STM32H7_FDCAN2

		/* Call stm32_fdcansockinitialize() to get an instance of the CAN2 interface */

		ret = stm32_fdcansockinitialize(1);
		if (ret < 0)
			{
			canerr("ERROR:  Failed to get FDCAN interface %d\n", ret);
			return ret;
			}

		/* test the socket */
		socketcan_test(1);

		/* Now we are initialized */
		initialized++;
		#endif
	}

	return (initialized != 0) ? OK : ERROR;
}

#endif /* CONFIG_CAN */
