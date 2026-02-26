/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
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
#pragma once


#include "../../../../sophgo/sg2000_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             0
#define PX4_NUMBER_I2C_BUSES      	2

#define GPIO_OUTPUT_SET             1
#define GPIO_OUTPUT_CLEAR           0

#include <chip.h>
#include <sg2000_gpio.h>
#include <sg2000_i2c.h>
#include <sg2000_spi.h>
#include <arch/board/board_memorymap.h>

#include "riscv_mmu.h"

/*
 *  PX4 uses the words in bigendian order MSB to LSB
 *   word  [0]    [1]    [2]   [3]
 *   bits 127:96  95-64  63-32, 31-00,
 */
#define PX4_CPU_UUID_BYTE_LENGTH                16
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* Battery backed up SRAM definitions */
#define PX4_BBSRAM_SIZE             2048

/* define common formating across all commands */

#define PX4_CPU_UUID_WORD32_FORMAT              "%08x"
#define PX4_CPU_UUID_WORD32_SEPARATOR           ":"

#define PX4_CPU_UUID_WORD32_UNIQUE_H            3 /* Least significant digits change the most */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            2 /* Middle High significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_L            1 /* Middle Low significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_N            0 /* Most significant digits change the least */

static inline struct spi_dev_s *board_spibus_initialize_wrap(int port)
{
	return sg2000_spibus_initialize(port - 1);
}

#define PX4_BUS_OFFSET       1                  /* MPFS buses are 0 based, so adjustment needed */
#define px4_savepanic(fileno, context, length)
#define px4_spibus_initialize(bus_num_1based)	sg2000_spibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#define px4_i2cbus_initialize(bus_num_1based)	sg2000_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)			sg2000_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)				sg2000_gpio_config(pinset)
#define px4_arch_unconfiggpio(pinset)			sg2000_gpio_config(pinset)
#define px4_arch_gpioread(pinset)				sg2000_gpio_read(pinset)
#define px4_arch_gpiowrite(pinset, value)		sg2000_gpio_write(pinset, value)
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)

#define PX4_MAKE_GPIO_INPUT(gpio) 	(0)
#define PX4_MAKE_GPIO_OUTPUT(gpio) 	(0)
#define PX4_GPIO_PIN_OFF(gpio) 		(0)

/* SG2000 can access HRT timer counter directly from user space in
 * CONFIG_BUILD_PROTECTED and CONFIG_BUILD_KERNEL
 */

#define PX4_USERSPACE_HRT
static inline uintptr_t hrt_absolute_time_usr_base(void)
{
	return 0;
}

#  define px4_cache_aligned_data()
#  define px4_cache_aligned_alloc malloc

__END_DECLS
