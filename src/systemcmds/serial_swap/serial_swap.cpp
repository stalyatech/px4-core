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
 * @file serial_swap.cpp
 *
 * Swap or restore the TX and RX pins of a serial port (TIOCSSWAP).
 *
 * The UART keeps the setting across later opens and baud rate changes, so
 * it can be applied before or after the user of the port starts.
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <nuttx/serial/tioctl.h>

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Swap or restore the TX and RX pins of a serial port.

### Example
$ serial_swap /dev/ttyS2 on
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("serial_swap", "command");
	PRINT_MODULE_USAGE_ARG("<device>", "Serial device, e.g. /dev/ttyS2", false);
	PRINT_MODULE_USAGE_ARG("on|off", "Swap or restore TX and RX", false);
}

extern "C" __EXPORT int serial_swap_main(int argc, char *argv[])
{
	if (argc != 3 || (strcmp(argv[2], "on") != 0 && strcmp(argv[2], "off") != 0)) {
		usage();
		return 1;
	}

	const bool swap = (strcmp(argv[2], "on") == 0);

	const int fd = open(argv[1], O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("cannot open %s", argv[1]);
		return 1;
	}

	const int ret = ioctl(fd, TIOCSSWAP, swap ? SER_SWAP_ENABLED : 0);
	close(fd);

	if (ret < 0) {
		PX4_ERR("%s: TIOCSSWAP failed (the port may not support it)", argv[1]);
		return 1;
	}

	PX4_INFO("%s TX/RX %s", argv[1], swap ? "swapped" : "restored");
	return 0;
}
