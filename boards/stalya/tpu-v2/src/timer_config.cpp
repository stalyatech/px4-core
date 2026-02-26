/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file timer_config.cpp
 *
 * TPU-v2 PWM rate-group mapping for SG2000 PX4IO.
 *
 * SG2000 PX4IO currently drives PWM via /dev/pwmX lower-halves, not the
 * io_timer backend. This table provides the logical "timer group" layout
 * expected by PX4IO register handling:
 *
 * Group 0: channels 1..4 (bitmask 0x0F)
 * Group 1: channels 5..6 (bitmask 0x30)
 * Group 2: channels 7..8 (bitmask 0xC0)
 * Group 3: unused
 */

#include <stdint.h>

extern "C" {

const unsigned g_sg2000_pwm_group_count = 3;

const uint32_t g_sg2000_pwm_group_masks[4] = {
	0x0Fu, /* ch1..ch4 */
	0x30u, /* ch5..ch6 */
	0xC0u, /* ch7..ch8 */
	0x00u, /* unused */
};

} // extern "C"
