/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "NanoRadarMR72.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace mr72
{

NanoRadarMR72 *g_dev{nullptr};

static int start(const char *port, uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	if (port == nullptr) {
		PX4_ERR("serial port required");
		return PX4_ERROR;
	}

	// Instantiate the driver.
	g_dev = new NanoRadarMR72(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the NanoRadar MR72 radar.

Most boards are configured to enable/start the driver on a specified UART using the SENS_MR72_CFG parameter.

### Examples

Attempt to start driver on a specified serial device.
$ mr72 start -d /dev/ttyS5
Stop driver
$ mr72 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mr72", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS0", "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 25, "Sensor rotation - forward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace mr72

extern "C" __EXPORT int mr72_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
	const char *port = nullptr;
	const char *myoptarg = nullptr;
	int myoptind = 1;
	int ch = 0;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			port = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option");
			return mr72::usage();
		}
	}

	if (myoptind >= argc) {
		return mr72::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return mr72::start(port, rotation);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return mr72::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return mr72::status();
	}

	return mr72::usage();
}
