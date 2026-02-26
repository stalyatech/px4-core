/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tpu_ppm.c
 *
 * Register SG2000 PPM lower-half for PX4IO RC input path.
 */

#include <px4_platform_common/px4_config.h>
#include "board_config.h"

#include <errno.h>
#include <syslog.h>

#include <nuttx/timers/capture.h>

#if !defined(CONFIG_SG2000_PPM)
#  error "TPU-V2 PX4IO requires CONFIG_SG2000_PPM=y"
#endif

#if !defined(CONFIG_SG2000_PPM_GPIO_PIN)
#  error "TPU-V2 PX4IO requires CONFIG_SG2000_PPM_GPIO_PIN"
#endif

#include <arch/sg2000/ppm.h>

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][ppm] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

int board_ppm_setup(void)
{
	PX4IO_DBG("setup begin (gpio pin=%d)\n", CONFIG_SG2000_PPM_GPIO_PIN);
	struct cap_lowerhalf_s *lower = sg2000_ppm_initialize(CONFIG_SG2000_PPM_GPIO_PIN);

	if (lower == NULL) {
		PX4IO_DBG("sg2000_ppm_initialize failed\n");
		return -ENODEV;
	}

	const int ret = cap_register("/dev/capture0", lower);
	if (ret < 0 && ret != -EEXIST) {
		PX4IO_DBG("cap_register failed: %d\n", ret);
		return ret;
	}

	PX4IO_DBG("capture device /dev/capture0 %s\n", (ret == -EEXIST) ? "exists" : "registered");
	return OK;
}
