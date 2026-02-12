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
 * @file led.c
 *
 * Icicle board LED backend.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "chip.h"
#include <sapphire_gpio.h>
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

#ifdef CONFIG_ARCH_LEDS
static bool nuttx_owns_leds = true;
//                                B  R  S  G
//                                0  1  2  3
static const uint8_t xlatpx4[] = {1, 2, 4, 0};
#  define xlat(p) xlatpx4[(p)]
static uint32_t g_ledmap[] = {
	GPIO_LED_GREEN,   // Indexed by BOARD_LED_GREEN
	GPIO_LED_BLUE,    // Indexed by BOARD_LED_BLUE
	GPIO_LED_RED,     // Indexed by BOARD_LED_RED
	GPIO_LED_SAFETY,  // Indexed by LED_SAFETY by xlatpx4
};

#else

#  define xlat(p) (p)
static uint32_t g_ledmap[] = {
	GPIO_LED_BLUE,   	// Indexed by LED_BLUE
	GPIO_LED_RED,     // Indexed by LED_RED, LED_AMBER
	0,                // Indexed by LED_SAFETY (defaulted to an input)
	GPIO_LED_GREEN,   // Indexed by LED_GREEN
};

#endif

/****************************************************************************
 * Name: phy_set_led
 *
 * Set the state of an LED.
 *
 * @param led the LED to set.
 * @param state the state to set the LED to.
 *
 ****************************************************************************/

 static void phy_set_led(int led, bool state)
{
	sapphire_gpio_write(BOARD_GET_PRT(g_ledmap[led]),
									BOARD_GET_PIN(g_ledmap[led]), state);
}

/****************************************************************************
 * Name: phy_get_led
 *
 * Get the state of an LED.
 *
 * @param led the LED to get the state for.
 *
 * @return the state of the LED.
 *
 ****************************************************************************/

 static bool phy_get_led(int led)
{
	return sapphire_gpio_read(BOARD_GET_PRT(g_ledmap[led]),
												BOARD_GET_PIN(g_ledmap[led]));
}

/****************************************************************************
 * Name: led_init
 *
 * Initialize the LED backend.
 *
 * This function should be called once during system initialization
 * to configure the LEDs.
 *
 ****************************************************************************/

 __EXPORT void led_init(void)
{
	/* Configure LED GPIOs for output */

	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++)
		{
			sapphire_gpio_config(BOARD_GET_PRT(g_ledmap[l]),
											 BOARD_GET_PIN(g_ledmap[l]),
													 GPIO_PINMODE_OUTPUT);
		}
}

/****************************************************************************
 * Name: led_on
 *
 * Turn an LED on.
 *
 * @param led the LED to turn on.
 *
 ****************************************************************************/

 __EXPORT void led_on(int led)
{
	phy_set_led(led, true);
}

/****************************************************************************
 * Name: led_off
 *
 * Turn an LED off.
 *
 * @param led the LED to turn off.
 *
 ****************************************************************************/

 __EXPORT void led_off(int led)
{
	phy_set_led(led, false);
}

/****************************************************************************
 * Name: led_toggle
 *
 * Toggle an LED on or off.
 *
 * @param led the LED to toggle.
 *
 * Toggles the state of the LED: if it was on, it will be
 * turned off and vice versa.
 *
 ****************************************************************************/

 __EXPORT void led_toggle(int led)
{
	phy_set_led(led, !phy_get_led(led));
}

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
	led_init();
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
	if (!nuttx_owns_leds) {
		return;
	}

	switch (led) {
	default:
		break;

	case LED_HEAPALLOCATE:
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_IRQSENABLED:
		phy_set_led(BOARD_LED_BLUE, false);
		phy_set_led(BOARD_LED_GREEN, true);
		break;

	case LED_STACKCREATED:
		phy_set_led(BOARD_LED_GREEN, true);
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_INIRQ:
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_SIGNAL:
		phy_set_led(BOARD_LED_GREEN, true);
		break;

	case LED_ASSERTION:
		phy_set_led(BOARD_LED_RED, true);
		phy_set_led(BOARD_LED_BLUE, true);
		break;

	case LED_PANIC:
		phy_set_led(BOARD_LED_RED, true);
		break;

	case LED_IDLE : /* IDLE */
		phy_set_led(BOARD_LED_RED, true);
		break;
	}
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
	if (!nuttx_owns_leds) {
		return;
	}

	switch (led) {
	default:
		break;

	case LED_SIGNAL:
		phy_set_led(BOARD_LED_GREEN, false);
		break;

	case LED_INIRQ:
		phy_set_led(BOARD_LED_BLUE, false);
		break;

	case LED_ASSERTION:
		phy_set_led(BOARD_LED_RED, false);
		phy_set_led(BOARD_LED_BLUE, false);
		break;

	case LED_PANIC:
		phy_set_led(BOARD_LED_RED, false);
		break;

	case LED_IDLE : /* IDLE */
		phy_set_led(BOARD_LED_RED, false);
		break;
	}
}

#endif /* CONFIG_ARCH_LEDS */
