/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * SG2000 board-level early startup hooks for IO-NEX2.
 */

#include <px4_platform_common/px4_config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>

#include <arch/board/board.h>

__BEGIN_DECLS
int board_pwm_setup(void);
__END_DECLS

__EXPORT void sg2000_boardinitialize(void)
{
	syslog(LOG_INFO, "tpu-v2: sg2000_boardinitialize\n");
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;

	if (board_pwm_setup() != OK) {
		syslog(LOG_ERR, "tpu-v2: board_pwm_setup failed\n");
	}

	return OK;
}
