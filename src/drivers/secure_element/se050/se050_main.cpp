/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file se050_main.cpp
 *
 * PX4 front-end for the NuttX NXP SE05X secure element driver.
 * Registers /dev/se05x on the configured I2C bus.
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* NuttX crypto/se05x.h and the STM32H7 I2C bus initializer are plain C;
 * wrap them for C++ linkage.
 */
extern "C" {
#include <nuttx/crypto/se05x.h>
	struct i2c_master_s *stm32_i2cbus_initialize(int port);
}

#ifndef SE050_DEFAULT_I2C_BUS
#  define SE050_DEFAULT_I2C_BUS 2
#endif

#ifndef SE050_DEFAULT_ADDRESS
#  define SE050_DEFAULT_ADDRESS 0x48
#endif

#ifndef SE050_DEFAULT_FREQUENCY
#  define SE050_DEFAULT_FREQUENCY 400000
#endif

#ifndef SE050_DEVPATH
#  define SE050_DEVPATH "/dev/se05x"
#endif

static bool g_registered = false;
static struct se05x_config_s g_cfg;

static int se050_start(int bus, uint8_t addr, uint32_t freq)
{
	if (g_registered) {
		PX4_INFO("already registered");
		return 0;
	}

	struct i2c_master_s *i2c = stm32_i2cbus_initialize(bus);

	if (i2c == nullptr) {
		PX4_ERR("stm32_i2cbus_initialize(%d) failed", bus);
		return -ENODEV;
	}

	g_cfg.address        = addr;
	g_cfg.frequency      = freq;
	g_cfg.set_enable_pin = nullptr;  /* ENA is tied to VCC on nexus-v1 */

	int ret = se05x_register(SE050_DEVPATH, i2c, &g_cfg);

	if (ret < 0) {
		PX4_ERR("se05x_register: %d", ret);
		return ret;
	}

	g_registered = true;
	PX4_INFO("%s on I2C%d @ 0x%02x (%lu Hz xfers)",
		 SE050_DEVPATH, bus, (unsigned)addr, (unsigned long)freq);
	return 0;
}

static int se050_status()
{
	int fd = open(SE050_DEVPATH, O_RDWR);

	if (fd < 0) {
		PX4_INFO("%s not registered (errno=%d)", SE050_DEVPATH, errno);
		return 0;
	}

	struct se05x_info_s info = {};
	struct se05x_uid_s uid = {};

	int ret = ioctl(fd, SEIOC_GET_INFO, (unsigned long)&info);

	if (ret == 0) {
		PX4_INFO("OEF-ID: 0x%04x", info.oef_id);

	} else {
		PX4_WARN("SEIOC_GET_INFO: errno=%d", errno);
	}

	ret = ioctl(fd, SEIOC_GET_UID, (unsigned long)&uid);

	if (ret == 0) {
		char hex[SE05X_MODULE_UNIQUE_ID_LEN * 2 + 1];

		for (size_t i = 0; i < SE05X_MODULE_UNIQUE_ID_LEN; i++) {
			snprintf(&hex[i * 2], 3, "%02x", uid.uid[i]);
		}

		PX4_INFO("UID: %s", hex);
	}

	close(fd);
	return 0;
}

static int se050_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
NXP SE050 secure element front-end. Registers the NuttX /dev/se05x
character device on the given I2C bus, then applications use standard
open/ioctl(SEIOC_*) calls to perform key, signature and attestation
operations.

Must be started AFTER any slow (100 kHz-only) devices on the same bus
have finished their one-shot initialisation — SE050 transfers use a
per-message 400 kHz clock override.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("se050", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Register /dev/se05x");
	PRINT_MODULE_USAGE_PARAM_INT('b', SE050_DEFAULT_I2C_BUS, 0, 6, "I2C bus number", true);
	PRINT_MODULE_USAGE_PARAM_INT('a', SE050_DEFAULT_ADDRESS, 0, 0x7f, "I2C slave address", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', SE050_DEFAULT_FREQUENCY, 100000, 1000000, "I2C bus frequency (Hz)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print OEF-ID and UID");
	return 1;
}

extern "C" __EXPORT int se050_main(int argc, char *argv[])
{
	if (argc < 2) {
		return se050_usage("missing command");
	}

	const char *cmd = argv[1];

	if (!strcmp(cmd, "status")) {
		return se050_status();
	}

	if (!strcmp(cmd, "start")) {
		int bus = SE050_DEFAULT_I2C_BUS;
		int addr = SE050_DEFAULT_ADDRESS;
		int freq = SE050_DEFAULT_FREQUENCY;

		int ch;
		int opt_index = 1;
		const char *opt_arg = nullptr;

		while ((ch = px4_getopt(argc, argv, "b:a:f:", &opt_index, &opt_arg)) != EOF) {
			switch (ch) {
			case 'b': bus = atoi(opt_arg); break;
			case 'a': addr = (int)strtol(opt_arg, nullptr, 0); break;
			case 'f': freq = atoi(opt_arg); break;
			default: return se050_usage("unknown option");
			}
		}

		return se050_start(bus, (uint8_t)addr, (uint32_t)freq);
	}

	return se050_usage("unknown command");
}
