/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file fdcan.c
 *
 * Board-specific CAN functions.
 */

#if !defined(CONFIG_CAN)

#include <stdint.h>

#include "board_config.h"


__EXPORT
uint16_t board_get_can_interfaces(void)
{
	uint16_t enabled_interfaces = 0x3;

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_CAN2)) {
		enabled_interfaces &= ~(1 << 1);
	}

	return enabled_interfaces;
}

#else

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_fdcan_sock.h"
#include "board_config.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
int can_devinit(void);

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
	static int initialized = 0;
	int ret;

	/* Check if we have already initialized */
	if (initialized == 0) {

		#ifdef CONFIG_STM32H7_FDCAN1

		/* Call stm32_fdcansockinitialize() to get an instance of the FDCAN interface */

		ret = stm32_fdcansockinitialize(0);
		if (ret < 0)
			{
			canerr("ERROR:  Failed to get FDCAN interface %d\n", ret);
			return ret;
			}

		/* Now we are initialized */
		initialized++;
		#endif

		#ifdef CONFIG_STM32H7_FDCAN2

		/* Call stm32_fdcansockinitialize() to get an instance of the CAN2 interface */

		ret = stm32_fdcansockinitialize(1);
		if (ret < 0)
			{
			canerr("ERROR:  Failed to get FDCAN interface %d\n", ret);
			return ret;
			}

		/* Now we are initialized */
		initialized++;
		#endif
	}

	return (initialized != 0) ? OK : ERROR;
}

#endif /* CONFIG_CAN */
