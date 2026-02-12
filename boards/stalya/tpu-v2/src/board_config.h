/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Nexus IO-NEX2 internal definitions (SG2000 / RISC-V).
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* TPU-v2 RISC-V bring-up uses software-mapped output channels for now. */

#define DIRECT_PWM_OUTPUT_CHANNELS	8
#define BOARD_NUM_IO_TIMERS 		0
#define BOARD_HAS_NO_CAPTURE
#define BOARD_HAS_HW_VERSIONING		1

#include <px4_platform_common/board_common.h>
