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
#include <stdbool.h>
#include <stdint.h>

/* TPU-v2 RISC-V PX4IO firmware transport on SG2000 little core. */

#define PX4FMU_SERIAL_DEVICE		"/dev/ttyS0"
#define PX4FMU_SERIAL_BAUDRATE		1562500

/* Global PX4IO debug switch: comment this out to disable PX4IO debug logs. */
#define PX4IO_DEBUG 1

/* Board-specific control/status signals on TPU-v2.
 *
 * LED_BLUE is wired to PWR_GPIO[6] (LED_STATUS).  px4iofirmware's
 * heartbeat_blink() toggles it every 250 ms via LED_BLUE(); the helper
 * lives in led_status.c.  The other LEDs aren't routed on this board.
 */
__BEGIN_DECLS
void tpu_v2_led_status_set(bool on);
__END_DECLS

#define LED_AMBER(_on_true)		do { (void)(_on_true); } while (0)
#define LED_SAFETY(_on_true)	do { (void)(_on_true); } while (0)
#define LED_BLUE(_on_true)		tpu_v2_led_status_set(_on_true)
#define LED_GREEN(_on_true)		do { (void)(_on_true); } while (0)

/* PX4IO firmware exposes 8 servo channels. On TPU-v2 these are physically
 * routed to SG2000 HW PWM15..PWM8 — HW PWM0..PWM7 drive board LEDs and
 * other peripherals (e.g. /dev/pwm0 = breath LED).  The servo→HW mapping
 * lives in tpu_pwm.c (g_sg2000_pwm_servo_hw_channels override).
 */
#define DIRECT_PWM_OUTPUT_CHANNELS 8

#define BOARD_NUM_IO_TIMERS 		0
#define BOARD_HAS_NO_CAPTURE
#define BOARD_HAS_HW_VERSIONING		1

#include <px4_platform_common/board_common.h>
