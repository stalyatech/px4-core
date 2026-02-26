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

#include <board_config.h>
#include <stdint.h>
#include <errno.h>
#include <drivers/drv_adc.h>
#include <syslog.h>

#if defined(CONFIG_SG2000_ADC)
extern "C" {
#include <sg2000_adc.h>
}
#endif

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][adc] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

int px4_arch_adc_init(uint32_t base_address)
{
	(void)base_address;
#if defined(CONFIG_SG2000_ADC)
	const int ret = sg2000_adc_init();
	PX4IO_DBG("init: ret=%d\n", ret);
	return ret;
#else
	PX4IO_DBG("init: CONFIG_SG2000_ADC disabled\n");
	return -ENODEV;
#endif
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	(void)base_address;
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	(void)base_address;
#if defined(CONFIG_SG2000_ADC)
	uint16_t value = 0;

	if (sg2000_adc_read((uint8_t)channel, &value) != OK) {
		PX4IO_DBG("sample ch=%u failed\n", channel);
		return UINT32_MAX;
	}

	return value;
#else
	(void)channel;
	return UINT32_MAX;
#endif
}

float px4_arch_adc_reference_v()
{
#ifdef BOARD_ADC_POS_REF_V
	return BOARD_ADC_POS_REF_V;
#else
	return 3.3f;
#endif
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12;
}
