/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Nexus IO-NEX2 internal definitions (SG2000 / RISC-V).
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>

/* TPU-v2 RISC-V PX4IO firmware transport on SG2000 little core. */

#define PX4FMU_SERIAL_DEVICE		"/dev/ttyS0"
#define PX4FMU_SERIAL_BAUDRATE		1562500

/* RC: PPM via /dev/capture0 (XGPIOB_13 / VIVO_D8, flat GPIO 45) plus SBUS
 * on UART1 (/dev/ttyS1).  The SBUS RX line is inverted by an external
 * board-level inverter, so the SG2000 UART driver receives standard
 * non-inverted SBUS framing (100 kbps, 8E2) — no in-driver TIOCSINVERT
 * needed (and the SG2000 driver does not implement it anyway).
 */
#define PX4IO_SBUS_DEVICE		"/dev/ttyS1"

/* RC I/O mux pins (TPU-v2 schematic).
 *
 * UART1_TX = XGPIOA_10 (SD0_D1 pad, route 3) → SBUS_OUT when SBUS_OUT_EN=0.
 * UART1_RX = XGPIOA_11 (SD0_D2 pad, route 3) ← SBUS_IN or DSM_IN, selected
 *                                              by RC_INPUT_SEL.
 *
 * SBUS_OUT_EN (XGPIOA_13, SD0_CD pad, flat GPIO 13):
 *   1 → external SBUS_OUT driver disabled, RSSI ADC3 input valid.
 *   0 → external SBUS_OUT driver enabled, RSSI value invalid.
 *
 * RC_INPUT_SEL (XGPIOB_22, VIVO_CLK pad, flat GPIO 32+22=54):
 *   0 → SBUS receiver routed to UART1_RX.
 *   1 → DSM   receiver routed to UART1_RX.
 */
#define SBUS_OUT_EN_GPIO_PIN		13U
#define RC_INPUT_SEL_GPIO_PIN		54U
#define PPM_GPIO_PIN				45U

/* PX4IO ADC channel mapping for TPU-v2 hardware.
 *
 * src/modules/px4iofirmware/sg2000/adc.cpp computes
 *   sg2000_channel = channel - ADC_VSERVO
 * with sg2000 ch 0,1,2 → ADC1,ADC2,ADC3.  This board wires:
 *   ADC2 = VSERVO (sg2000 ch 1) → ADC_VSERVO=5
 *   ADC3 = RSSI   (sg2000 ch 2) → ADC_RSSI=6
 * Override the px4io.h defaults (4/5) which would map to ADC1/ADC2.
 */
#undef ADC_VSERVO
#undef ADC_RSSI
#define ADC_VSERVO			5U
#define ADC_RSSI			6U

/* Global PX4IO debug switch: comment this out to disable PX4IO debug logs. */
#define PX4IO_DEBUG 1

/* Board-specific control/status signals on TPU-v2.
 *
 * LED_BLUE is wired to PWR_GPIO[6] (LED_STATUS).  px4iofirmware's
 * heartbeat_blink() toggles it every 250 ms via LED_BLUE(); the helper
 * lives in led_status.c.  The other LEDs aren't routed on this board.
 */
__BEGIN_DECLS
void tpu_v2_led_status_set(bool on);

/* RC I/O mux helpers (rc_mux.c).
 *
 * tpu_v2_rc_mux_init() sets pinmux + DDR + safe defaults
 * (SBUS_OUT_EN=1, RC_INPUT_SEL=0).  Called from board_app_initialize()
 * before PWM/PPM bring-up.
 *
 * tpu_v2_sbus_out_enable(true)  → drive SBUS_OUT pin (RSSI invalid).
 * tpu_v2_sbus_out_enable(false) → release SBUS_OUT, RSSI ADC3 valid.
 *
 * tpu_v2_rc_input_select_dsm(true)  → UART1_RX listens to DSM receiver.
 * tpu_v2_rc_input_select_dsm(false) → UART1_RX listens to SBUS receiver.
 */
int  tpu_v2_rc_mux_init(void);
void tpu_v2_sbus_out_enable(bool on);
void tpu_v2_rc_input_select_dsm(bool dsm);
bool tpu_v2_rc_input_is_dsm(void);
__END_DECLS

/* px4io.h sees this and skips its default no-op definition. */
#define ENABLE_SBUS_OUT(_s)		tpu_v2_sbus_out_enable((bool)(_s))

/* Tells px4iofirmware/registers.c that this board exposes a hardware mux
 * to switch UART1_RX between SBUS and DSM receivers, controlled via the
 * PX4IO_P_SETUP_RC_INPUT_SEL setup register written by the FMU.
 */
#define BOARD_HAS_RC_INPUT_SEL		1

#define LED_AMBER(_on_true)		do { (void)(_on_true); } while (0)
#define LED_SAFETY(_on_true)	do { (void)(_on_true); } while (0)
#define LED_BLUE(_on_true)		tpu_v2_led_status_set(_on_true)
#define LED_GREEN(_on_true)		do { (void)(_on_true); } while (0)

/* PX4IO firmware exposes 8 servo channels. On TPU-v2 these are physically
 * routed to SG2000 HW PWM15..PWM8 — HW PWM0..PWM7 drive board LEDs and
 * other peripherals (e.g. /dev/pwm0 = breath LED).  The servo→HW mapping
 * lives in tpu_pwm.c (g_sg2000_pwm_servo_hw_channels override).
 */
#define DIRECT_PWM_OUTPUT_CHANNELS 8

#define BOARD_NUM_IO_TIMERS 		0
#define BOARD_HAS_NO_CAPTURE
#define BOARD_HAS_HW_VERSIONING		1

#include <px4_platform_common/board_common.h>
