/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 *   Author: Volvox <volvox@stalya.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * nexus-v2: PX4 on the soft FCU of the Ti375C529, started by Linux over
 * remoteproc. There is no I/O coprocessor on this board: the FCU drives every
 * output itself, through blocks in the FPGA fabric.
 *
 * P0 brings up the core only, so most of the I/O is not described here yet.
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include "board_type.h"

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define BOARD_REVISIONS {{"Nexus FC V2", '1', NULL}, \
												 {"Nexus FC V2", '1', NULL}}

/* LEDs on GPIO0 of the soft SoC */

#define GPIO_LED_RED						BOARD_MAKE_PIN(0, 1, -1)
#define GPIO_LED_GREEN					BOARD_MAKE_PIN(0, 2, -1)
#define GPIO_LED_BLUE						BOARD_MAKE_PIN(0, 3, -1)

#define GPIO_LED_SAFETY 				BOARD_MAKE_PIN(0, 4, -1)
#define GPIO_BTN_SAFETY 				BOARD_MAKE_PIN(0, 0, -1)

#define PX4_GPIO_INIT_LIST { \
	}

#define BOARD_HAS_ON_RESET 				1

/* The console is a pair of ring buffers in memory Linux also reads
 * (CONFIG_SAPPHIRE_RAMCON), so let PX4 buffer its own output as well.
 */

#define BOARD_ENABLE_CONSOLE_BUFFER

/* No timer-driven outputs yet: PWM, DShot and RC come from fabric blocks in
 * P3, together with the UART block that carries GPS, telemetry and RC.
 */

#define BOARD_NUM_IO_TIMERS 			0
#define DIRECT_PWM_OUTPUT_CHANNELS		0

#define BOARD_SPI_BUS_MAX_BUS_ITEMS 	1

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

extern void	board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
