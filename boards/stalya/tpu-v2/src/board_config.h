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

/* TPU-v2 RISC-V PX4IO firmware transport on SG2000 little core. */

#define PX4FMU_SERIAL_DEVICE		"/dev/ttyS4"
#define PX4FMU_SERIAL_BAUDRATE		1500000

/* Board-specific control/status signals are currently optional on TPU-v2. */
#define LED_AMBER(_on_true)		do { (void)(_on_true); } while (0)
#define LED_SAFETY(_on_true)	do { (void)(_on_true); } while (0)
#define LED_BLUE(_on_true)		do { (void)(_on_true); } while (0)
#define LED_GREEN(_on_true)		do { (void)(_on_true); } while (0)

#define DIRECT_PWM_OUTPUT_CHANNELS	8
#define BOARD_NUM_IO_TIMERS 		0
#define BOARD_HAS_NO_CAPTURE
#define BOARD_HAS_HW_VERSIONING		1

#include <px4_platform_common/board_common.h>
