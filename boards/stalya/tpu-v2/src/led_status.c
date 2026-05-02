/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file led_status.c
 *
 * LED indicator support for the TPU-v2 IO board:
 *
 *   - LED_STATUS = PWR_GPIO[6] is wired to the px4iofirmware LED_BLUE
 *     macro.  The firmware's heartbeat_blink() toggles it every 250 ms,
 *     so the LED ends up at the standard PX4IO 1 Hz heartbeat cadence
 *     when the FMU link is up — no thread of our own needed.
 *     tpu_v2_led_status_set() is the helper LED_BLUE() resolves to;
 *     it lazily configures DDR=output on first call and writes DR.
 *
 *   - LED_BREATH = /dev/pwm0 (HW PWM0 -> XGPIOB[0]) is run by a
 *     dedicated high-priority RR worker that drives a smooth breathe:
 *         0..1000 ms : fade in   (duty 0%   -> 100%)
 *         1000..2000 : fade out  (duty 100% -> 0%)
 *         2000..2500 : blank     (duty 0%, LED dark)
 *     Total cycle = 2500 ms.  Independent busy-wait threads can't
 *     share CPU on this SG2000 SKU (no preemptive timer source for
 *     SCHED_RR slice end on the C906 LITTLE core), so we stay in a
 *     single thread.  The cycle only runs while
 *     PX4IO_P_STATUS_FLAGS_FMU_OK is set in r_status_flags; with no
 *     FMU link the LED stays dark and the cycle restarts cleanly from
 *     fade-in once the link recovers.
 *
 * The boot stage (sg2000_boardinitialize -> board_led_boot_toggle) first
 * drives XGPIOB[0] in raw GPIO mode for a few quick blinks; once
 * board_pwm_setup() runs the PWM peripheral pinmux is applied and the
 * breath worker takes the pin over via /dev/pwm0.
 *
 * PWM_8..15 are intentionally left alone here — they are application
 * (servo) channels driven by the regular px4iofirmware path.
 */

#include <nuttx/config.h>
#include "board_config.h"

#include <errno.h>
#include <fcntl.h>
/* NuttX PWM upper-half duty: 0..0xFFFF (16-bit, 0xFFFF = 100%).  ub16_t
 * is uint32_t but the PWM API only uses the low 16 bits per its header.
 */
#define PWM_DUTY_FULL              0xFFFFU
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/timers/pwm.h>

#include <modules/px4iofirmware/protocol.h>

/* px4iofirmware status page (defined in registers.c).  Reading
 * r_page_status[PX4IO_P_STATUS_FLAGS] from this thread is safe: it's a
 * 16-bit aligned volatile load, mixer.cpp updates the FMU_OK bit on a
 * 500 ms timeout (FMU_INPUT_DROP_LIMIT_US), and a stale read by one
 * busy_step (~25 ms) doesn't matter for an LED.
 */
extern volatile uint16_t r_page_status[];

/* PWR_GPIO controller (always-on power island). */
#define PWR_GPIO_BASE              0x05021000UL
#define DW_GPIO_SWPORTA_DR_OFFSET  0x00
#define DW_GPIO_SWPORTA_DDR_OFFSET 0x04

/* PWR/RTC pinmux for PWR_GPIO[n] — function-select bits [2:0] of
 * RTC_FMUX_GPIO_REG_IOCTRL_<n>; 0 = native GPIO function.
 */
#define PWR_PINMUX_BASE            0x05027000UL
#define PWR_PINMUX_FUNC_GPIO       0u

#define LED_STATUS_PIN             6U          /* PWR_GPIO[6] */

#define BREATH_DEVPATH             "/dev/pwm0" /* HW PWM0 -> XGPIOB[0] */
#define PWM_FREQUENCY              1000U

/* Breath cycle quantum (~25 ms @ 425 MHz, -Os).  Tuned together with the
 * step counts below so that fade-in + fade-out + blank = 2500 ms.
 */
#define BREATH_STEP_LOOPS          2500000ULL
#define BREATH_FADE_IN_STEPS       20U   /* 500 ms */
#define BREATH_FADE_OUT_STEPS      20U   /* 500 ms */
#define BREATH_BLANK_STEPS          4U   /* 100 ms */
#define BREATH_CYCLE_STEPS         (BREATH_FADE_IN_STEPS + BREATH_FADE_OUT_STEPS + BREATH_BLANK_STEPS)

static inline volatile uint32_t *pwr_gpio_reg(uintptr_t off)
{
	return (volatile uint32_t *)(uintptr_t)(PWR_GPIO_BASE + off);
}

/* Public helper used by px4iofirmware's LED_BLUE() macro.
 *
 * Re-asserts pinmux=GPIO and DDR=output every call (idempotent RMW, ~ns
 * cost at 4 Hz).  PWR_GPIO[6] sits on the always-on PWR/RTC island whose
 * controller can be perturbed by unrelated PWR-domain activity (RTC
 * SARADC, cmdqu wakeup, suspend/resume) — without this re-arm the pin
 * silently drops to input after such a glitch and the LED stays dark
 * even though heartbeat_blink() keeps writing DR.
 */
void tpu_v2_led_status_set(bool on)
{
	volatile uint32_t *pinmux =
		(volatile uint32_t *)(uintptr_t)(PWR_PINMUX_BASE + 4u * LED_STATUS_PIN);
	*pinmux = (*pinmux & ~0x7u) | (PWR_PINMUX_FUNC_GPIO & 0x7u);

	volatile uint32_t *ddr = pwr_gpio_reg(DW_GPIO_SWPORTA_DDR_OFFSET);
	*ddr = *ddr | (1u << LED_STATUS_PIN);

	volatile uint32_t *dr = pwr_gpio_reg(DW_GPIO_SWPORTA_DR_OFFSET);

	if (on) {
		*dr = *dr | (1u << LED_STATUS_PIN);

	} else {
		*dr = *dr & ~(1u << LED_STATUS_PIN);
	}
}

static void busy_step(void)
{
	for (volatile uint64_t i = 0; i < BREATH_STEP_LOOPS; i++) { /* busy */ }
}

static inline bool fmu_link_ok(void)
{
	return (r_page_status[PX4IO_P_STATUS_FLAGS] & PX4IO_P_STATUS_FLAGS_FMU_OK) != 0;
}

static int pwm_open_and_start(const char *path, ub16_t duty)
{
	int fd = open(path, O_RDWR);

	if (fd < 0) {
		syslog(LOG_ERR, "tpu-v2: open %s failed (%d)\n", path, errno);
		return -1;
	}

	struct pwm_info_s info = { .frequency = PWM_FREQUENCY, .duty = duty };

	if (ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info) < 0) {
		syslog(LOG_ERR, "tpu-v2: %s SETCHARACTERISTICS failed (%d)\n", path, errno);
		close(fd);
		return -1;
	}

	if (ioctl(fd, PWMIOC_START, 0) < 0) {
		syslog(LOG_ERR, "tpu-v2: %s START failed (%d)\n", path, errno);
		close(fd);
		return -1;
	}

	return fd;
}

static void *breath_worker(void *arg)
{
	(void)arg;

	/* Take over XGPIOB[0] from the boot-time GPIO toggle by opening
	 * /dev/pwm0 (board_pwm_setup() already moved the pinmux to the
	 * PWM peripheral function before we got here).
	 */
	const int breath_fd = pwm_open_and_start(BREATH_DEVPATH, 0);

	if (breath_fd < 0) {
		return NULL;
	}

	unsigned step = 0;
	struct pwm_info_s info = { .frequency = PWM_FREQUENCY, .duty = 0 };

	for (;;) {
		uint32_t duty;
		const bool link_ok = fmu_link_ok();

		if (!link_ok) {
			duty = 0;
			step = 0;

		} else if (step < BREATH_FADE_IN_STEPS) {
			duty = (step * PWM_DUTY_FULL) / BREATH_FADE_IN_STEPS;

		} else if (step < BREATH_FADE_IN_STEPS + BREATH_FADE_OUT_STEPS) {
			const unsigned out = step - BREATH_FADE_IN_STEPS;
			duty = ((BREATH_FADE_OUT_STEPS - out) * PWM_DUTY_FULL) /
			       BREATH_FADE_OUT_STEPS;

		} else {
			duty = 0;
		}

		info.duty = (ub16_t)duty;
		(void)ioctl(breath_fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);

		busy_step();
		(void)sched_yield();

		if (link_ok) {
			step = (step + 1U) % BREATH_CYCLE_STEPS;
		}
	}

	return NULL;
}

int board_led_status_start(void)
{
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 4096);

	/* Run at the SAME priority as the user_start init thread (default).
	 * SCHED_RR with equal priority + the explicit sched_yield() in the
	 * busy-wait loop lets us cooperate with the firmware main loop:
	 * neither thread starves, heartbeat_blink() runs, FMU<->IO comms
	 * runs, and the breath duty still gets refreshed each step.
	 * LED_STATUS is driven by px4iofirmware via LED_BLUE().
	 */
	struct sched_param sp = { .sched_priority = SCHED_PRIORITY_DEFAULT };
	(void)pthread_attr_setschedpolicy(&attr, SCHED_RR);
	(void)pthread_attr_setschedparam(&attr, &sp);
	(void)pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	pthread_t thread;
	const int ret = pthread_create(&thread, &attr, breath_worker, NULL);
	pthread_attr_destroy(&attr);

	if (ret != 0) {
		syslog(LOG_ERR, "tpu-v2: breath pthread_create failed (%d)\n", ret);
		return -ret;
	}

	(void)pthread_detach(thread);
	return OK;
}
