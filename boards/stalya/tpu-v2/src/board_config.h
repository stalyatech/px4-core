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

#define PX4FMU_SERIAL_DEVICE		"/dev/ttyS0"
#define PX4FMU_SERIAL_BAUDRATE		1500000

/* Global PX4IO debug switch: comment this out to disable PX4IO debug logs. */
#define PX4IO_DEBUG 1

/* Board-specific control/status signals are currently optional on TPU-v2. */
#define LED_AMBER(_on_true)		do { (void)(_on_true); } while (0)
#define LED_SAFETY(_on_true)	do { (void)(_on_true); } while (0)
#define LED_BLUE(_on_true)		do { (void)(_on_true); } while (0)
#define LED_GREEN(_on_true)		do { (void)(_on_true); } while (0)

/* Derive logical PWM channel count from active NuttX SG2000 PWM instances. */
#ifdef CONFIG_SG2000_PWM0
# define TPU_PWM0_ENABLED 1
#else
# define TPU_PWM0_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM1
# define TPU_PWM1_ENABLED 1
#else
# define TPU_PWM1_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM2
# define TPU_PWM2_ENABLED 1
#else
# define TPU_PWM2_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM3
# define TPU_PWM3_ENABLED 1
#else
# define TPU_PWM3_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM4
# define TPU_PWM4_ENABLED 1
#else
# define TPU_PWM4_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM5
# define TPU_PWM5_ENABLED 1
#else
# define TPU_PWM5_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM6
# define TPU_PWM6_ENABLED 1
#else
# define TPU_PWM6_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM7
# define TPU_PWM7_ENABLED 1
#else
# define TPU_PWM7_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM8
# define TPU_PWM8_ENABLED 1
#else
# define TPU_PWM8_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM9
# define TPU_PWM9_ENABLED 1
#else
# define TPU_PWM9_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM10
# define TPU_PWM10_ENABLED 1
#else
# define TPU_PWM10_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM11
# define TPU_PWM11_ENABLED 1
#else
# define TPU_PWM11_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM12
# define TPU_PWM12_ENABLED 1
#else
# define TPU_PWM12_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM13
# define TPU_PWM13_ENABLED 1
#else
# define TPU_PWM13_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM14
# define TPU_PWM14_ENABLED 1
#else
# define TPU_PWM14_ENABLED 0
#endif
#ifdef CONFIG_SG2000_PWM15
# define TPU_PWM15_ENABLED 1
#else
# define TPU_PWM15_ENABLED 0
#endif

#define DIRECT_PWM_OUTPUT_CHANNELS \
	(TPU_PWM0_ENABLED + TPU_PWM1_ENABLED + TPU_PWM2_ENABLED + TPU_PWM3_ENABLED + \
	 TPU_PWM4_ENABLED + TPU_PWM5_ENABLED + TPU_PWM6_ENABLED + TPU_PWM7_ENABLED + \
	 TPU_PWM8_ENABLED + TPU_PWM9_ENABLED + TPU_PWM10_ENABLED + TPU_PWM11_ENABLED + \
	 TPU_PWM12_ENABLED + TPU_PWM13_ENABLED + TPU_PWM14_ENABLED + TPU_PWM15_ENABLED)

#define BOARD_NUM_IO_TIMERS 		0
#define BOARD_HAS_NO_CAPTURE
#define BOARD_HAS_HW_VERSIONING		1

#include <px4_platform_common/board_common.h>
