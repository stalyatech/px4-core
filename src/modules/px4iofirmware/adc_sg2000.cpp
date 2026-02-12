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

extern "C" int sg2000_adc_init(void);
extern "C" int sg2000_adc_read(uint8_t channel, uint16_t *value);

int adc_init(void)
{
	const int ret = sg2000_adc_init();

	if (ret != OK) {
		syslog(LOG_ERR, "px4io_sg2000: adc init failed (%d)\n", ret);
	}

	return ret;
}

uint16_t adc_measure(unsigned channel)
{
	/* Keep legacy PX4IO channel IDs and map to SG2000 channels. */
	if (channel < ADC_VSERVO) {
		return 0;
	}

	const uint8_t sg2000_channel = static_cast<uint8_t>(channel - ADC_VSERVO);
	uint16_t value = 0;

	if (sg2000_adc_read(sg2000_channel, &value) != OK) {
		return 0;
	}

	return value;
}
