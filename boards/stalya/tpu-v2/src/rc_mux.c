/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file rc_mux.c
 *
 * TPU-v2 RC I/O mux: SBUS_OUT enable + DSM/SBUS input selection.
 *
 *   XGPIOA_13 (SD0_CD pad)   = SBUS_OUT_EN — 1 disables external SBUS_OUT
 *                              driver and exposes the RSSI line on ADC3,
 *                              0 enables SBUS_OUT (RSSI invalid).
 *   XGPIOB_22 (VIVO_CLK pad) = RC_INPUT_SEL — 0 routes SBUS receiver to
 *                              UART1_RX, 1 routes DSM receiver instead.
 *
 * Both pads are bonded on CV181x but were missing from upstream NuttX
 * sg2000_pinmux.h; the new SG2000_PINMUX_SD0_CD / SG2000_PINMUX_VIVO_CLK
 * defines and matching XGPIO func 3 entries live there now.
 */

#include <px4_platform_common/px4_config.h>
#include "board_config.h"

#include <errno.h>
#include <syslog.h>

#include <arch/sg2000/gpio.h>
#include <arch/sg2000/pinmux.h>

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][rc_mux] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

/* Pinmux register offsets and func 3 (XGPIO) selector for the two pads.
 * Mirror of SG2000_PINMUX_SD0_CD / SG2000_PINMUX_VIVO_CLK in
 * arch/risc-v/src/sg2000/hardware/sg2000_pinmux.h — duplicated here so this
 * board file does not need to reach into the arch-private header.
 */
#define TPU_V2_PINMUX_SD0_CD            0x034u
#define TPU_V2_PINMUX_VIVO_CLK          0x160u
#define TPU_V2_FUNC_SD0_CD_XGPIOA_13    3u
#define TPU_V2_FUNC_VIVO_CLK_XGPIOB_22  3u

static const struct sg2000_pinmux_cfg_s g_rc_mux_pinmux[] = {
	/* SBUS_OUT_EN: pad has no peripheral function we use, so periph_func
	 * is set to the same XGPIO func 3 — sg2000_pinmux_apply() switches on
	 * mode anyway and we always call it with MODE_GPIO.
	 */
	{ TPU_V2_PINMUX_SD0_CD,   TPU_V2_FUNC_SD0_CD_XGPIOA_13,   TPU_V2_FUNC_SD0_CD_XGPIOA_13   },
	{ TPU_V2_PINMUX_VIVO_CLK, TPU_V2_FUNC_VIVO_CLK_XGPIOB_22, TPU_V2_FUNC_VIVO_CLK_XGPIOB_22 },
};

static bool g_rc_input_dsm = false;

int tpu_v2_rc_mux_init(void)
{
	int ret = sg2000_pinmux_apply(g_rc_mux_pinmux,
				      sizeof(g_rc_mux_pinmux) / sizeof(g_rc_mux_pinmux[0]),
				      SG2000_PINMUX_MODE_GPIO);
	if (ret < 0) {
		PX4IO_DBG("pinmux_apply failed: %d\n", ret);
		return ret;
	}

	ret = sg2000_gpio_config(SBUS_OUT_EN_GPIO_PIN | SG2000_GPIO_OUTPUT);
	if (ret < 0) {
		PX4IO_DBG("config SBUS_OUT_EN failed: %d\n", ret);
		return ret;
	}

	ret = sg2000_gpio_config(RC_INPUT_SEL_GPIO_PIN | SG2000_GPIO_OUTPUT);
	if (ret < 0) {
		PX4IO_DBG("config RC_INPUT_SEL failed: %d\n", ret);
		return ret;
	}

	/* Safe defaults: SBUS_OUT driver disabled (RSSI valid), SBUS input. */
	sg2000_gpio_write(SBUS_OUT_EN_GPIO_PIN, true);
	sg2000_gpio_write(RC_INPUT_SEL_GPIO_PIN, false);
	g_rc_input_dsm = false;

	PX4IO_DBG("init OK: SBUS_OUT_EN=1 (RSSI valid), RC_INPUT_SEL=0 (SBUS)\n");
	return OK;
}

void tpu_v2_sbus_out_enable(bool on)
{
	/* on=true  -> SBUS_OUT_EN=0 (drive UART1_TX onto SBUS_OUT, RSSI invalid)
	 * on=false -> SBUS_OUT_EN=1 (release SBUS_OUT, RSSI ADC3 valid)
	 */
	sg2000_gpio_write(SBUS_OUT_EN_GPIO_PIN, !on);
	PX4IO_DBG("sbus_out=%d (pin=%d)\n", (int)on, !on);
}

void tpu_v2_rc_input_select_dsm(bool dsm)
{
	sg2000_gpio_write(RC_INPUT_SEL_GPIO_PIN, dsm);
	g_rc_input_dsm = dsm;
	PX4IO_DBG("rc_input=%s\n", dsm ? "DSM" : "SBUS");
}

bool tpu_v2_rc_input_is_dsm(void)
{
	return g_rc_input_dsm;
}
