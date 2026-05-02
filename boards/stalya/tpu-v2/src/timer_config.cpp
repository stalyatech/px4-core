/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file timer_config.cpp
 *
 * Synthetic PWM "timer" descriptor for the TPU-v2 IO board, consumed only by
 * the PX4 actuators-metadata generator at build time
 * (Tools/module_config/output_groups_from_timer_config.py).
 *
 * The real SG2000 hardware has 4 PWM controllers × 4 channels each:
 *   TIM2 -> HW PWM 0..3   (board LEDs / breath)
 *   TIM3 -> HW PWM 4..7   (unused on TPU-v2)
 *   TIM4 -> HW PWM 8..11  (servos 4..7 via tpu_pwm.c override)
 *   TIM5 -> HW PWM 12..15 (servos 0..3 via tpu_pwm.c override)
 *
 * The PX4IO protocol exposes 4 PWM rate groups (PX4IO_P_SETUP_PWM_RATE_GROUP0..3).
 * The TPU-v2 servo→HW mapping in tpu_pwm.c is {15,14,13,12,11,10,9,8}, and
 * tpu_tmr.cpp groups the 8 servo channels as {0,1}, {2,3}, {4,5}, {6,7}.  To
 * match that — and to give QGC four independent PWM rate sliders in the
 * Actuators page — we expose four logical "timers" here, each owning 2
 * adjacent servo channels.  The Timer:: enum values below have no relation
 * to SG2000 HW; they are only required to be distinct so the generator's
 * groupby() yields four groups.
 *
 * This file is intentionally NOT compiled into firmware (it is excluded from
 * boards/stalya/tpu-v2/src/CMakeLists.txt).  Only the build-time Python
 * parser reads it.
 */

#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1), // logical group 0 -> servos 0,1 (HW PWM15,14 on TIM5)
	initIOTimer(Timer::Timer2), // logical group 1 -> servos 2,3 (HW PWM13,12 on TIM5)
	initIOTimer(Timer::Timer3), // logical group 2 -> servos 4,5 (HW PWM11,10 on TIM4)
	initIOTimer(Timer::Timer4), // logical group 3 -> servos 6,7 (HW PWM9,8  on TIM4)
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}), // servo 0
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}), // servo 1
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin2}), // servo 2
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel2}, {GPIO::PortA, GPIO::Pin3}), // servo 3
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortA, GPIO::Pin4}), // servo 4
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortA, GPIO::Pin5}), // servo 5
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortA, GPIO::Pin6}), // servo 6
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortA, GPIO::Pin7}), // servo 7
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
