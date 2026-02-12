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

__EXPORT void sg2000_boardinitialize(void)
{
	syslog(LOG_INFO, "tpu-v2: sg2000_boardinitialize\n");
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;
	return OK;
}
