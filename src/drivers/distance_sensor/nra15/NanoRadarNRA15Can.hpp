/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Technology. All rights reserved.
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
 * @file NanoRadarNRA15Can.hpp
 *
 * Nanoradar NRA15 altimeter radar over CAN (SocketCAN on NuttX).
 *
 * A dedicated task blocks on the CAN socket and stamps every frame on
 * arrival. A cycle starts with the target status (0x70B) and is complete
 * once the announced number of target info frames (0x70C) has arrived.
 */

#pragma once

#include <nuttx/can.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>

#include "nra15_can_protocol.hpp"

using namespace time_literals;

class NanoRadarNRA15Can : public ModuleBase<NanoRadarNRA15Can>
{
public:
	/**
	 * @param bus       0 for CAN1 (can0), 1 for CAN2 (can1)
	 * @param rotation  distance_sensor orientation
	 */
	NanoRadarNRA15Can(int bus, uint8_t rotation);
	~NanoRadarNRA15Can() override;

	static int task_spawn(int argc, char *argv[]);
	static NanoRadarNRA15Can *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	static constexpr float MIN_DISTANCE{0.1f};     // m
	static constexpr float MAX_DISTANCE{100.0f};   // m
	static constexpr float FIELD_OF_VIEW{37.0f};   // deg
	static constexpr hrt_abstime DATA_TIMEOUT{200_ms};  // 25 ms cycle
	static constexpr hrt_abstime VERSION_RETRY{1_s};
	static constexpr int MAX_TARGETS{4};           // kept for print_status

	void handle_frame(const struct can_frame &frame, hrt_abstime now);
	void start_cycle(const nra15_can::TargetStatus &status, hrt_abstime now);
	void add_target(const nra15_can::TargetInfo &target);
	void publish_cycle(bool complete);
	void request_version();

	const int _bus;

	int _sock{-1};
	bool _bitrate_set{false};

	// Current cycle
	bool _in_cycle{false};
	hrt_abstime _cycle_start{0};
	uint8_t _expected{0};
	uint8_t _received{0};
	float _nearest{-1.0f};

	// Last complete cycle, for print_status
	nra15_can::TargetInfo _targets[MAX_TARGETS] {};
	uint8_t _num_targets{0};
	float _last_nearest{-1.0f};

	bool _have_roll_count{false};
	uint8_t _roll_count{0};

	bool _have_version{false};
	uint8_t _version[3] {};
	hrt_abstime _version_requested{0};

	hrt_abstime _last_frame{0};
	hrt_abstime _last_cycle{0};

	PX4Rangefinder _px4_rangefinder;

	perf_counter_t _frames_perf{perf_alloc(PC_COUNT, MODULE_NAME": frames")};
	perf_counter_t _cycles_perf{perf_alloc(PC_COUNT, MODULE_NAME": cycles")};
	perf_counter_t _missed_perf{perf_alloc(PC_COUNT, MODULE_NAME": missed cycles")};
	perf_counter_t _incomplete_perf{perf_alloc(PC_COUNT, MODULE_NAME": incomplete cycles")};
	perf_counter_t _timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": data timeouts")};
	perf_counter_t _read_err_perf{perf_alloc(PC_COUNT, MODULE_NAME": read errors")};
};
