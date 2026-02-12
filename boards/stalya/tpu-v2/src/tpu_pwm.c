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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/timers/pwm.h>

#include <sg2000_pwm.h>

int board_pwm_setup(void)
{
	for (int channel = 0; channel < 8; channel++) {
		struct pwm_lowerhalf_s *pwm = sg2000_pwminitialize(channel);

		if (pwm == NULL) {
			return -ENODEV;
		}

		char devpath[16];
		snprintf(devpath, sizeof(devpath), "/dev/pwm%d", channel);
		int ret = pwm_register(devpath, pwm);

		if (ret < 0 && ret != -EEXIST) {
			return ret;
		}
	}

	return OK;
}
