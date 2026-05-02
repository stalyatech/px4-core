/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/*
 * @file pwm_servo.c
 *
 * SG2000 PWM servo bridge for PX4 up_pwm_servo_* API.
 */

#include <px4_platform_common/px4_config.h>

#include <drivers/drv_pwm_output.h>

#include <nuttx/timers/pwm.h>

#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifndef DIRECT_PWM_OUTPUT_CHANNELS
#define DIRECT_PWM_OUTPUT_CHANNELS 8
#endif

#define SG2000_PWM_GROUP_MAX 4
#define SG2000_PWM_GROUP_DEFAULT_COUNT 4

static int g_pwm_fd[DIRECT_PWM_OUTPUT_CHANNELS];
static uint16_t g_pwm_value[DIRECT_PWM_OUTPUT_CHANNELS];
static unsigned g_group_rate_hz[SG2000_PWM_GROUP_MAX] = {50, 50, 50, 50};
static bool g_armed;
static bool g_fds_initialized;

/* Weak defaults: boards can override these from their own board timer mapping source file. */
__attribute__((weak)) const unsigned g_sg2000_pwm_group_count = SG2000_PWM_GROUP_DEFAULT_COUNT;
__attribute__((weak)) const uint32_t g_sg2000_pwm_group_masks[SG2000_PWM_GROUP_MAX] = {
	0x03u, /* ch1..ch2 */
	0x0Cu, /* ch3..ch4 */
	0x30u, /* ch5..ch6 */
	0xC0u  /* ch7..ch8 */
};

/* Mapping from PX4 servo channel index (0..DIRECT_PWM_OUTPUT_CHANNELS-1) to
 * the SG2000 HW PWM channel number used to construct the /dev/pwmN path.
 * Default is identity (servo n -> HW PWM n); boards override when servos are
 * physically wired to a non-zero-based subset (e.g. TPU-v2 uses HW8..15).
 */
__attribute__((weak)) const uint8_t g_sg2000_pwm_servo_hw_channels[DIRECT_PWM_OUTPUT_CHANNELS] = {
#if DIRECT_PWM_OUTPUT_CHANNELS >= 1
	0,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 2
	1,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 3
	2,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 4
	3,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 5
	4,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 6
	5,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 7
	6,
#endif
#if DIRECT_PWM_OUTPUT_CHANNELS >= 8
	7,
#endif
};

static inline uint32_t group_mask(unsigned group)
{
	if (group >= SG2000_PWM_GROUP_MAX) {
		return 0;
	}

	if (group >= g_sg2000_pwm_group_count) {
		return 0;
	}

	return g_sg2000_pwm_group_masks[group];
}

static inline unsigned channel_group(unsigned channel)
{
	const uint32_t bit = (1u << channel);

	for (unsigned group = 0; group < SG2000_PWM_GROUP_MAX; group++) {
		if (group_mask(group) & bit) {
			return group;
		}
	}

	/* No configured group: caller will reject with rate_hz == 0. */
	return SG2000_PWM_GROUP_MAX;
}

static int apply_channel(unsigned channel)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS || g_pwm_fd[channel] < 0) {
		return -1;
	}

	const unsigned group = channel_group(channel);
	const unsigned rate_hz = (group < SG2000_PWM_GROUP_MAX) ? g_group_rate_hz[group] : 0;

	if (rate_hz == 0) {
		return -1;
	}

	const uint32_t period_us = 1000000u / rate_hz;
	uint16_t pulse_us = g_pwm_value[channel];

	if (pulse_us > period_us) {
		pulse_us = (uint16_t)period_us;
	}

	struct pwm_info_s pwm;
	memset(&pwm, 0, sizeof(pwm));
	pwm.frequency = rate_hz;
	pwm.duty = (ub16_t)(((uint64_t)pulse_us << 16) / period_us);

	if (ioctl(g_pwm_fd[channel], PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm)) < 0) {
		return -1;
	}

	return ioctl(g_pwm_fd[channel], PWMIOC_START, 0);
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	if (!g_fds_initialized) {
		for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
			g_pwm_fd[i] = -1;
		}

		g_fds_initialized = true;
	}

	for (unsigned channel = 0; channel < DIRECT_PWM_OUTPUT_CHANNELS; channel++) {
		if (g_pwm_fd[channel] >= 0) {
			close(g_pwm_fd[channel]);
			g_pwm_fd[channel] = -1;
		}

		if (channel_mask & (1u << channel)) {
			const unsigned hw_channel = g_sg2000_pwm_servo_hw_channels[channel];
			char devpath[16];
			snprintf(devpath, sizeof(devpath), "/dev/pwm%u", hw_channel);
			g_pwm_fd[channel] = open(devpath, O_RDONLY);
		}
	}

	g_armed = false;
	return OK;
}

void up_pwm_servo_deinit(uint32_t channel_mask)
{
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set(unsigned channel, uint16_t value)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS) {
		return -1;
	}

	g_pwm_value[channel] = value;
	return g_armed ? apply_channel(channel) : OK;
}

void up_pwm_update(unsigned channels_mask)
{
	if (!g_armed) {
		return;
	}

	for (unsigned channel = 0; channel < DIRECT_PWM_OUTPUT_CHANNELS; channel++) {
		if (channels_mask & (1u << channel)) {
			(void)apply_channel(channel);
		}
	}
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	if (group >= SG2000_PWM_GROUP_MAX) {
		return -1;
	}

	if (group >= g_sg2000_pwm_group_count) {
		return -1;
	}

	if (rate == 0 || rate > 10000) {
		return -1;
	}

	g_group_rate_hz[group] = rate;

	if (g_armed) {
		const uint32_t mask = group_mask(group);

		for (unsigned channel = 0; channel < DIRECT_PWM_OUTPUT_CHANNELS; channel++) {
			if (mask & (1u << channel)) {
				(void)apply_channel(channel);
			}
		}
	}

	return OK;
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	return group_mask(group);
}

void up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	g_armed = armed;

	for (unsigned channel = 0; channel < DIRECT_PWM_OUTPUT_CHANNELS; channel++) {
		if (!(channel_mask & (1u << channel)) || g_pwm_fd[channel] < 0) {
			continue;
		}

		if (armed) {
			(void)apply_channel(channel);

		} else {
			(void)ioctl(g_pwm_fd[channel], PWMIOC_STOP, 0);
		}
	}
}
