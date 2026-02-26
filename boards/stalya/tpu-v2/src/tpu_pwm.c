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
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/timers/pwm.h>

#include <sg2000_pwm.h>

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][pwm] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

#if defined(CONFIG_SG2000_PWM0) || defined(CONFIG_SG2000_PWM1) || defined(CONFIG_SG2000_PWM2) || \
	defined(CONFIG_SG2000_PWM3) || defined(CONFIG_SG2000_PWM4) || defined(CONFIG_SG2000_PWM5) || \
	defined(CONFIG_SG2000_PWM6) || defined(CONFIG_SG2000_PWM7) || defined(CONFIG_SG2000_PWM8) || \
	defined(CONFIG_SG2000_PWM9) || defined(CONFIG_SG2000_PWM10) || defined(CONFIG_SG2000_PWM11) || \
	defined(CONFIG_SG2000_PWM12) || defined(CONFIG_SG2000_PWM13) || defined(CONFIG_SG2000_PWM14) || \
	defined(CONFIG_SG2000_PWM15)
static const uint8_t g_pwm_channels[] = {
#ifdef CONFIG_SG2000_PWM0
	0,
#endif
#ifdef CONFIG_SG2000_PWM1
	1,
#endif
#ifdef CONFIG_SG2000_PWM2
	2,
#endif
#ifdef CONFIG_SG2000_PWM3
	3,
#endif
#ifdef CONFIG_SG2000_PWM4
	4,
#endif
#ifdef CONFIG_SG2000_PWM5
	5,
#endif
#ifdef CONFIG_SG2000_PWM6
	6,
#endif
#ifdef CONFIG_SG2000_PWM7
	7,
#endif
#ifdef CONFIG_SG2000_PWM8
	8,
#endif
#ifdef CONFIG_SG2000_PWM9
	9,
#endif
#ifdef CONFIG_SG2000_PWM10
	10,
#endif
#ifdef CONFIG_SG2000_PWM11
	11,
#endif
#ifdef CONFIG_SG2000_PWM12
	12,
#endif
#ifdef CONFIG_SG2000_PWM13
	13,
#endif
#ifdef CONFIG_SG2000_PWM14
	14,
#endif
#ifdef CONFIG_SG2000_PWM15
	15,
#endif
};
# define TPU_PWM_CHANNEL_COUNT (sizeof(g_pwm_channels))
#else
# define TPU_PWM_CHANNEL_COUNT 0
#endif

int board_pwm_setup(void)
{
	PX4IO_DBG("setup begin\n");

	if (TPU_PWM_CHANNEL_COUNT != DIRECT_PWM_OUTPUT_CHANNELS) {
		syslog(LOG_ERR, "tpu-v2: PWM config mismatch (DIRECT_PWM_OUTPUT_CHANNELS=%u, enabled=%u)\n",
		       (unsigned)DIRECT_PWM_OUTPUT_CHANNELS, (unsigned)TPU_PWM_CHANNEL_COUNT);
		return -EINVAL;
	}

	if (TPU_PWM_CHANNEL_COUNT == 0) {
		PX4IO_DBG("no PWM channels enabled by config\n");
		return -ENODEV;
	}

	for (size_t logical_channel = 0; logical_channel < TPU_PWM_CHANNEL_COUNT; logical_channel++) {
		const int hw_channel = g_pwm_channels[logical_channel];
		PX4IO_DBG("init channel logical=%u hw=%d\n", (unsigned)logical_channel, hw_channel);
		struct pwm_lowerhalf_s *pwm = sg2000_pwminitialize(hw_channel);

		if (pwm == NULL) {
			PX4IO_DBG("init hw channel %d failed: lowerhalf NULL\n", hw_channel);
			return -ENODEV;
		}

		char devpath[16];
		snprintf(devpath, sizeof(devpath), "/dev/pwm%u", (unsigned)logical_channel);
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
