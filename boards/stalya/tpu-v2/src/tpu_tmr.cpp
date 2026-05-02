/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tpu_tmr.cpp
 *
 * TPU-v2 PWM rate-group mapping for SG2000 PX4IO.
 *
 * SG2000 PX4IO currently drives PWM via /dev/pwmX lower-halves, not the
 * io_timer backend. This table provides the logical "timer group" layout
 * expected by PX4IO register handling:
 *
 * TPU-v2 builds logical PWM outputs from enabled CONFIG_SG2000_PWMx instances.
 * This file derives PX4 PWM rate groups from DIRECT_PWM_OUTPUT_CHANNELS so that
 * grouping automatically follows NuttX PWM activation.
 */

#include <board_config.h>
#include <stdint.h>

extern "C" {

/* TPU-v2 has 8 servo channels; PX4IO protocol exposes 4 rate groups, so we
 * pair adjacent channels per group.  The SG2000 PWM HW has independent
 * period/duty registers per channel ([sg2000_pwm.c]), so even when two
 * channels in a group share a HW controller (TIM2 or TIM3 here), they can
 * still run at independent rates — but PX4IO clients (mixer, gimbal, ESC)
 * typically configure rates per-group, so 2 channels per group is the
 * standard PX4 layout.
 *
 *   Group 0 -> servo {0, 1}  -> HW PWM {15, 14}  (both on TIM3)
 *   Group 1 -> servo {2, 3}  -> HW PWM {13, 12}  (both on TIM3)
 *   Group 2 -> servo {4, 5}  -> HW PWM {11, 10}  (both on TIM2)
 *   Group 3 -> servo {6, 7}  -> HW PWM {9,  8 }  (both on TIM2)
 */
static constexpr unsigned k_max_groups = 4;
static constexpr unsigned k_pwm_channels = DIRECT_PWM_OUTPUT_CHANNELS;
static constexpr unsigned k_channels_per_group = 2;
static constexpr unsigned k_group_count =
	(k_pwm_channels + k_channels_per_group - 1) / k_channels_per_group;

static_assert(k_group_count <= k_max_groups,
	      "PX4IO protocol exposes only 4 PWM rate groups");

static constexpr uint32_t group_mask(unsigned group)
{
	if (group >= k_group_count) {
		return 0;
	}

	const unsigned begin = group * k_channels_per_group;
	unsigned end = begin + k_channels_per_group;

	if (end > k_pwm_channels) {
		end = k_pwm_channels;
	}

	uint32_t mask = 0;

	for (unsigned i = begin; i < end; i++) {
		mask |= (1u << i);
	}

	return mask;
}

const unsigned g_sg2000_pwm_group_count = k_group_count;

const uint32_t g_sg2000_pwm_group_masks[4] = {
	group_mask(0),
	group_mask(1),
	group_mask(2),
	group_mask(3),
};

} // extern "C"
