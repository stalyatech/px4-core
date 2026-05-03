/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * SG2000 board-level early startup hooks for IO-NEX2.
 */

#include <px4_platform_common/px4_config.h>
#include "board_config.h"

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>
#include <arch/sg2000/gpio.h>
#ifdef CONFIG_SG2000_CMDQU
#include <arch/sg2000/cmdqu.h>
#endif
#ifdef CONFIG_SG2000_WDT
#include <arch/sg2000/wdt.h>
#endif

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][init] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

__BEGIN_DECLS
int board_pwm_setup(void);
int board_ppm_setup(void);
int board_led_status_start(void);
__END_DECLS

/* Early boot indicator. Drives TWO outputs in parallel so we can isolate
 * which path is alive:
 *   - PWR_GPIO[6] (LED_STATUS) via raw MMIO. NuttX's sg2000_gpio.c only
 *     exposes XGPIOA..XGPIOD, so we touch the always-on power-island
 *     GPIO controller directly (DesignWare GPIO register layout).
 *   - XGPIOB[0] (LED_BREATH GPIO mode) via NuttX SG2000 GPIO API. The
 *     PWM driver hasn't been initialized yet at board_early_initialize
 *     time, so the pin sits in its reset alt-function (GPIO on most
 *     CV180x SKUs). 5 fast blinks here = board_early_initialize reached.
 */
#define TPU_PWR_GPIO_BASE          0x05021000UL
#define TPU_PWR_GPIO_DR_OFFSET     0x00
#define TPU_PWR_GPIO_DDR_OFFSET    0x04
#define TPU_PWR_LED_PIN            6U

/* PWR/RTC domain pinmux (CV180x family RTC_FMUX_GPIO_REG_IOCTRL_*).
 * Each PWR_GPIO[n] pin has a 32-bit IOCTRL register at base + 4*n;
 * function-select bits [2:0] = 0 selects native GPIO mode on all
 * known SKUs.  If the LED stays dark after this fix, the base or
 * offset layout differs on this part and needs the datasheet value.
 */
#define TPU_PWR_PINMUX_BASE        0x05027000UL
#define TPU_PWR_PINMUX_FUNC_GPIO   0u

static inline volatile uint32_t *tpu_pwr_gpio_reg(uintptr_t off)
{
	return (volatile uint32_t *)(uintptr_t)(TPU_PWR_GPIO_BASE + off);
}

static void board_led_boot_toggle(void)
{
	/* PWR_GPIO[6]: select GPIO function in PWR pinmux, then DDR=output. */
	volatile uint32_t *pinmux =
		(volatile uint32_t *)(uintptr_t)(TPU_PWR_PINMUX_BASE + 4u * TPU_PWR_LED_PIN);
	*pinmux = (*pinmux & ~0x7u) | (TPU_PWR_PINMUX_FUNC_GPIO & 0x7u);

	volatile uint32_t *ddr = tpu_pwr_gpio_reg(TPU_PWR_GPIO_DDR_OFFSET);
	*ddr = *ddr | (1u << TPU_PWR_LED_PIN);
	volatile uint32_t *dr  = tpu_pwr_gpio_reg(TPU_PWR_GPIO_DR_OFFSET);

	/* Keep this short: it is a busy-wait at board_early_initialize time
	 * before the FMU UART/RX path is up, so every ms here directly eats
	 * into the FMU px4io probe budget (700 ms non-deferred / 1000 ms
	 * deferred, src/drivers/px4io/px4io.cpp).  3 × 2 × 50 ms = 300 ms
	 * is enough to be visually unmistakable without starving the probe.
	 */
	for (int i = 0; i < 5; i++) {
		*dr = *dr ^ (1u << TPU_PWR_LED_PIN);
		up_udelay(50 * 1000);
	}
}

__EXPORT void sg2000_boardinitialize(void)
{
	board_led_boot_toggle();
	PX4IO_DBG("boardinitialize: enter\n");
}

__EXPORT void board_early_initialize(void)
{
	sg2000_boardinitialize();
}

__EXPORT void board_late_initialize(void)
{
	PX4IO_DBG("board_late_initialize: exit\n");
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;
	PX4IO_DBG("board_app_initialize: begin\n");
	PX4IO_DBG("FMU link: %s @ %u bps\n", PX4FMU_SERIAL_DEVICE, PX4FMU_SERIAL_BAUDRATE);

	/* Bring up the RC I/O mux pins (SBUS_OUT_EN, RC_INPUT_SEL) before any
	 * driver opens UART1 — boot defaults are SBUS_OUT off (RSSI valid) and
	 * SBUS input selected.  FMU later writes PX4IO_P_SETUP_RC_INPUT_SEL
	 * to switch to DSM if RC_INPUT_PROTO requests it.
	 */
	const int mux_ret = tpu_v2_rc_mux_init();

	if (mux_ret != OK) {
		syslog(LOG_ERR, "tpu-v2: tpu_v2_rc_mux_init failed (%d)\n", mux_ret);
	} else {
		PX4IO_DBG("tpu_v2_rc_mux_init: OK\n");
	}

	PX4IO_DBG("board_pwm_setup: start\n");
	if (board_pwm_setup() != OK) {
		syslog(LOG_ERR, "tpu-v2: board_pwm_setup failed\n");
	} else {
		PX4IO_DBG("board_pwm_setup: OK\n");
	}

	PX4IO_DBG("board_ppm_setup: start\n");
	const int ppm_ret = board_ppm_setup();

	if (ppm_ret != OK) {
		syslog(LOG_ERR, "tpu-v2: board_ppm_setup failed (%d), continuing\n", ppm_ret);
	} else {
		PX4IO_DBG("board_ppm_setup: OK\n");
	}

	/* Spawn heartbeat AFTER PWM/PPM nodes are registered so the worker
	 * can open /dev/pwm0 etc.  High-priority RR worker would otherwise
	 * monopolize the CPU and never let the registrations run.
	 */
	const int led_ret = board_led_status_start();

	if (led_ret != OK) {
		syslog(LOG_ERR, "tpu-v2: board_led_status_start failed (%d)\n", led_ret);
	} else {
		PX4IO_DBG("board_led_status_start: OK\n");
	}

	PX4IO_DBG("board_app_initialize: done\n");

#ifdef CONFIG_SG2000_CMDQU
	const int cmdqu_ret = sg2000_cmdqu_init();

	if (cmdqu_ret != OK) {
		syslog(LOG_ERR, "tpu-v2: sg2000_cmdqu_init failed (%d)\n", cmdqu_ret);
	}
#endif

#ifdef CONFIG_SG2000_WDT
	const int wdt_ret = sg2000_wdt_init();

	if (wdt_ret != OK && wdt_ret != -EEXIST) {
		syslog(LOG_ERR, "tpu-v2: sg2000_wdt_init failed (%d)\n", wdt_ret);
	} else {
		PX4IO_DBG("sg2000_wdt_init: %s\n", (wdt_ret == -EEXIST) ? "already registered" : "OK");
	}
#endif

	return OK;
}
