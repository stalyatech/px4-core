/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tpu_pwm.c
 *
 * Register SG2000 PWM lower-halves for PX4IO firmware usage.
 */

#include <nuttx/config.h>
#include "board_config.h"

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/timers/pwm.h>

#include <sg2000_pwm.h>

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][pwm] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

int board_pwm_setup(void)
{
	PX4IO_DBG("setup begin\n");

	for (int channel = 0; channel < 8; channel++) {
		PX4IO_DBG("init channel %d\n", channel);
		struct pwm_lowerhalf_s *pwm = sg2000_pwminitialize(channel);

		if (pwm == NULL) {
			PX4IO_DBG("init channel %d failed: lowerhalf NULL\n", channel);
			return -ENODEV;
		}

		char devpath[16];
		snprintf(devpath, sizeof(devpath), "/dev/pwm%d", channel);
		int ret = pwm_register(devpath, pwm);

		if (ret < 0 && ret != -EEXIST) {
			PX4IO_DBG("register %s failed: %d\n", devpath, ret);
			return ret;
		}

		PX4IO_DBG("register %s %s\n", devpath, (ret == -EEXIST) ? "exists" : "OK");
	}

	PX4IO_DBG("setup done\n");
	return OK;
}
