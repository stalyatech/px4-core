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

static constexpr unsigned k_max_groups = 4;
static constexpr unsigned k_pwm_channels = DIRECT_PWM_OUTPUT_CHANNELS;
static constexpr unsigned k_group_count =
	(k_pwm_channels == 0) ? 0 : (k_pwm_channels < k_max_groups ? k_pwm_channels : k_max_groups);

static constexpr uint32_t mask_range(unsigned begin, unsigned end)
{
	uint32_t mask = 0;

	for (unsigned i = begin; i < end; i++) {
		mask |= (1u << i);
	}

	return mask;
}

/* If channel count exceeds 4, group 3 holds channels [3..N-1]. */
static constexpr uint32_t group_mask(unsigned group)
{
	if (group >= k_group_count) {
		return 0;
	}

	if (group < 3) {
		return mask_range(group, group + 1);
	}

	return mask_range(3, k_pwm_channels);
}

const unsigned g_sg2000_pwm_group_count = k_group_count;

const uint32_t g_sg2000_pwm_group_masks[4] = {
	group_mask(0),
	group_mask(1),
	group_mask(2),
	group_mask(3),
};

} // extern "C"
