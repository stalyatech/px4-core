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

	if (TPU_PWM_CHANNEL_COUNT == 0) {
		PX4IO_DBG("no PWM channels enabled by config\n");
		return -ENODEV;
	}

	for (size_t i = 0; i < TPU_PWM_CHANNEL_COUNT; i++) {
		const unsigned hw_channel = g_pwm_channels[i];
		PX4IO_DBG("init hw channel %u\n", hw_channel);
		struct pwm_lowerhalf_s *pwm = sg2000_pwminitialize((int)hw_channel);

		if (pwm == NULL) {
			PX4IO_DBG("init hw channel %u failed: lowerhalf NULL\n", hw_channel);
			return -ENODEV;
		}

		/* devpath uses HW channel number directly so /dev/pwmN <-> SG2000 PWM-N.
		 * This keeps the breath LED on /dev/pwm0 (HW PWM0) while the PX4 servo
		 * mapping (HW PWM8..15) opens /dev/pwm8../dev/pwm15 — see
		 * g_sg2000_pwm_servo_hw_channels[] override below.
		 */
		char devpath[16];
		snprintf(devpath, sizeof(devpath), "/dev/pwm%u", hw_channel);
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

/* Override the platform-default servo-channel-to-HW-PWM-channel mapping.
 * On TPU-v2 the PX4 servo channels 0..7 are physically wired to SG2000
 * HW PWM15..PWM8.  HW PWM0 is the breath LED; HW PWM1..PWM3 are reserved
 * for other indicators.
 */
const uint8_t g_sg2000_pwm_servo_hw_channels[DIRECT_PWM_OUTPUT_CHANNELS] = {
	15, 14, 13, 12, 11, 10, 9, 8
};
