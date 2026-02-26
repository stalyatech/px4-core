/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file adc_sg2000.cpp
 *
 * Simple SG2000 ADC backend for PX4IO firmware.
 */

#include <px4_platform_common/px4_config.h>

#include <errno.h>
#include <stdint.h>
#include <syslog.h>

#include "px4io.h"

#if defined(CONFIG_SG2000_ADC)
extern "C" {
#include <sg2000_adc.h>
}
#endif

static bool g_adc_enabled = false;

int adc_init(void)
{
#if defined(CONFIG_SG2000_ADC)
	const int ret = sg2000_adc_init();

	if (ret != OK) {
		syslog(LOG_ERR, "px4io_sg2000: adc init failed (%d)\n", ret);
		g_adc_enabled = false;
		return ret;
	}

	g_adc_enabled = true;
	return OK;
#else
	g_adc_enabled = false;
	return OK;
#endif
}

uint16_t adc_measure(unsigned channel)
{
	/* Keep legacy PX4IO channel IDs and map to SG2000 channels. */
	if (channel < ADC_VSERVO) {
		return 0;
	}

	const uint8_t sg2000_channel = static_cast<uint8_t>(channel - ADC_VSERVO);
	uint16_t value = 0;

#if defined(CONFIG_SG2000_ADC)
	if (!g_adc_enabled) {
		return 0;
	}

	if (sg2000_adc_read(sg2000_channel, &value) != OK) {
		return 0;
	}
#else
	(void)sg2000_channel;
	return 0;
#endif

	return value;
}
