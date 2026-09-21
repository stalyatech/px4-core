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

#ifdef CONFIG_FS_PROCFS
	ret = mount(NULL, "/proc", "procfs", 0, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
		return ret;
	}

#endif /* CONFIG_FS_PROCFS */

	UNUSED(ret);
	return OK;
}
