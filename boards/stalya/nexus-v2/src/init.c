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
 * @file init.c
 *
 * nexus-v2 early startup. Linux loads and starts this firmware over
 * remoteproc; by the time anything here runs the DDR is already up.
 *
 * Code here runs before the rcS script; it starts the subsystems the board
 * needs. P0 brings up the core only, so there is no sensor bus, no storage
 * and no output stage yet.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <sys/mount.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <chip.h>
#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>

#ifdef CONFIG_SAPPHIRE_RPTUN
#  include <nuttx/serial/uart_rpmsg_raw.h>
#  include "sapphire_rptun.h"
#endif

#ifdef CONFIG_SAPPHIRE_I2C0
#  include <nuttx/i2c/i2c_master.h>
#  include "sapphire_i2c.h"
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/************************************************************************************
 * Name: board_peripheral_reset
 ************************************************************************************/

__EXPORT void board_peripheral_reset(int ms)
{
	syslog(LOG_DEBUG, "board_peripheral_reset\n");

	/* Nothing to reset yet: the peripherals arrive with the fabric blocks */
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 *   Called on entry to board_system_reset.
 *
 *   status - 1 if resetting to boot loader
 *            0 if just resetting
 *
 ************************************************************************************/

__EXPORT void board_on_reset(int status)
{
	syslog(LOG_DEBUG, "board_on_reset %d\n", status);
}

/************************************************************************************
 * Name: sapphire_boardinitialize
 *
 * Description:
 *   Called early, after memory is up and before any device is initialized.
 *
 ************************************************************************************/

__EXPORT void sapphire_boardinitialize(void)
{
	board_autoled_initialize();

	/* this call exists to fix a weird linking issue */

	up_udelay(0);
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Board specific initialization, reached through boardctl(BOARDIOC_INIT).
 *
 ****************************************************************************/

__EXPORT int board_app_initialize(uintptr_t arg)
{
	int ret;

	/* hrt first: everything below and the rcS script depend on it */

	px4_platform_init();

	/* initial LED state */

	drv_led_start();
	led_off(LED_RED);
	led_on(LED_GREEN);
	led_off(LED_BLUE);

	px4_platform_configure();

#ifdef CONFIG_SAPPHIRE_RPTUN
	/* The link to Linux: rpmsg over the vrings in the shared region. The tty
	 * on top of it is registered from rpmsg_serialrawinit() below, which the
	 * driver layer calls on its own.
	 */

	ret = sapphire_rptun_init(CONFIG_SAPPHIRE_RPTUN_CPUNAME);

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: rptun init failed: %d\n", ret);
	}

#endif /* CONFIG_SAPPHIRE_RPTUN */

#if defined(CONFIG_SAPPHIRE_I2C0) && defined(CONFIG_I2C_DRIVER)
	/* The dev board's own I2C bus (RTC PCF8523, EMC1413 temperature sensor,
	 * TCA9546A mux) as /dev/i2c0, for bring-up with the i2c tool.
	 */

	struct i2c_master_s *i2c0 = sapphire_i2cbus_initialize(0);

	if (i2c0 == NULL || i2c_register(i2c0, 0) < 0) {
		syslog(LOG_ERR, "ERROR: I2C0 init failed\n");
	}

#endif /* CONFIG_SAPPHIRE_I2C0 && CONFIG_I2C_DRIVER */

	/* /proc is mounted by px4_platform_init() above. */

	UNUSED(ret);
	return OK;
}

#ifdef CONFIG_RPMSG_UART_RAW
/****************************************************************************
 * Name: rpmsg_serialrawinit
 *
 * Description:
 *   Called by the driver layer to register the ttys carried over rpmsg. The
 *   peer name must be the one rptun knows Linux by; Linux sees this channel
 *   as /dev/ttyRPMSG0 once the endpoint is announced.
 *
 ****************************************************************************/

void rpmsg_serialrawinit(void)
{
	uart_rpmsg_raw_init(CONFIG_SAPPHIRE_RPTUN_CPUNAME, "RPMSG0", 4096, false);
}
#endif /* CONFIG_RPMSG_UART_RAW */
