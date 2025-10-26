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
 * Nexus internal definitions
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

/* Configuration ************************************************************************************/

#define BOARD_HAS_BUS_MANIFEST 			(1)
#define BOARD_HAS_HW_VERSIONING			(1)
#define HW_INFO_INIT_PREFIX     		"Nexus FC "

/* Efinix GPIOs *************************************************************************************/

#define GPIO_BTN_SAFETY 				BOARD_GPIO_PIN(0, 0, -1)

#define TONE_ALARM_PWM_OUT_PATH 		"/dev/pwm0"

/* LEDS */

#define GPIO_nLED_RED						BOARD_GPIO_PIN(0, 1, -1)
#define GPIO_nLED_GREEN					BOARD_GPIO_PIN(0, 2, -1)
#define GPIO_nLED_BLUE					BOARD_GPIO_PIN(0, 3, -1)

/* I2C */

#define I2C_RESET_SPEED         		100000
#define BOARD_I2C_BUS_CLOCK_INIT 		{100000, 100000}

/* RC Serial port */

#define RC_SERIAL_PORT					"/dev/ttyS2"
#define RC_SERIAL_SINGLEWIRE

#define BOARD_HAS_ON_RESET 				1

#define PX4_GPIO_INIT_LIST { \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 			0

#define BOARD_DSHOT_MOTOR_ASSIGNMENT 	{3, 2, 1, 0, 4, 5, 6, 7};

/* eMMC/SD */

#define SDIO_SLOTNO						0
#define SDIO_MINOR  					0

/* Battery ADC */

#define ADC_CHANNELS 					(1 << 1) | (1 << 2)
#define ADC_BATTERY_VOLTAGE_CHANNEL 	1
#define ADC_BATTERY_CURRENT_CHANNEL 	2

/* SPI */

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
extern int 	board_spinor_init(struct spi_dev_s *spinor);
extern void board_spidev_init(void);
extern int 	board_spibus_init(void);
extern int 	board_pwmdev_init(void);
extern int 	board_emmcsd_init(void);

#include <px4_platform_common/board_common.h>

const char *board_bl_version_string(void);
#define PX4_BL_VERSION board_bl_version_string()

#endif /* __ASSEMBLY__ */

__END_DECLS
