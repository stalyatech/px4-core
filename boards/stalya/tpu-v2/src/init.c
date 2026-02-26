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

#ifdef PX4IO_DEBUG
# define PX4IO_DBG(_fmt, ...) syslog(LOG_INFO, "[px4io][init] " _fmt, ##__VA_ARGS__)
#else
# define PX4IO_DBG(_fmt, ...)
#endif

__BEGIN_DECLS
int board_pwm_setup(void);
int board_ppm_setup(void);
__END_DECLS

/* Same LED pin used by milkv_duom_evb reference board appinit. */
#define TPU_BOOT_LED_GPIO_PIN 29

static void board_led_boot_toggle(void)
{
	const int gpio_init_ret = sg2000_gpio_initialize();

	if (gpio_init_ret < 0) {
		syslog(LOG_ERR, "tpu-v2: sg2000_gpio_initialize failed: %d\n", gpio_init_ret);
		return;
	}

	const int cfg_ret = sg2000_gpio_config((TPU_BOOT_LED_GPIO_PIN << SG2000_GPIO_PIN_SHIFT) |
					       SG2000_GPIO_OUTPUT);

	if (cfg_ret < 0) {
		syslog(LOG_ERR, "tpu-v2: sg2000_gpio_config pin %d failed: %d\n", TPU_BOOT_LED_GPIO_PIN, cfg_ret);
		return;
	}

	for (int i = 0; i < 3; i++) {
		sg2000_gpio_write(TPU_BOOT_LED_GPIO_PIN, true);
		up_udelay(100 * 1000);
		sg2000_gpio_write(TPU_BOOT_LED_GPIO_PIN, false);
		up_udelay(100 * 1000);
	}
}

__EXPORT void sg2000_boardinitialize(void)
{
	up_putc('\n');
	up_putc('[');
	up_putc('i');
	up_putc('o');
	up_putc(']');
	up_putc(' ');
	up_putc('b');
	up_putc('o');
	up_putc('o');
	up_putc('t');
	up_putc('\n');
	board_led_boot_toggle();
	syslog(LOG_INFO, "tpu-v2: sg2000_boardinitialize\n");
	PX4IO_DBG("boardinitialize: enter\n");
}

__EXPORT void board_late_initialize(void)
{
	up_putc('\n');
	up_putc('[');
	up_putc('L');
	up_putc(']');
	up_putc('\n');

	PX4IO_DBG("board_late_initialize: enter\n");
	board_led_boot_toggle();
	PX4IO_DBG("board_late_initialize: exit\n");
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;
	PX4IO_DBG("board_app_initialize: begin\n");
	PX4IO_DBG("FMU link: %s @ %u bps, debug console: /dev/ttyS1\n", PX4FMU_SERIAL_DEVICE, PX4FMU_SERIAL_BAUDRATE);

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

	PX4IO_DBG("board_app_initialize: done\n");
	return OK;
}
