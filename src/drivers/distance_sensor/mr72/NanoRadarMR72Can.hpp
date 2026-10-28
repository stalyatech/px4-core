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
 * @file NanoRadarMR72Can.hpp
 *
 * Nanoradar MR72 over CAN (SocketCAN on NuttX).
 *
 * A dedicated task blocks on the CAN socket and stamps every frame on
 * arrival, so the timestamps carry no polling delay. A measurement cycle
 * starts with 0x60A and is complete once the announced number of 0x60B
 * objects has arrived.
 */

#pragma once

#include <nuttx/can.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/radar_objects.h>
#include <uORB/topics/radar_status.h>

#include "mr72_can_protocol.hpp"

using namespace time_literals;

class NanoRadarMR72Can : public ModuleBase<NanoRadarMR72Can>
{
public:
	/**
	 * @param bus        0 for CAN1 (can0), 1 for CAN2 (can1)
	 * @param sensor_id  radar SensorID, 0...7
	 * @param rotation   distance_sensor orientation
	 * @param bitrate    CAN bitrate in bit/s
	 * @param raw        accept every frame and list the identifiers seen (bring-up)
	 * @param probe      send one empty frame on an unused identifier (bring-up)
	 */
	NanoRadarMR72Can(int bus, uint8_t sensor_id, uint8_t rotation, uint32_t bitrate, bool raw, bool probe);
	~NanoRadarMR72Can() override;

	static int task_spawn(int argc, char *argv[]);
	static NanoRadarMR72Can *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	static constexpr float MIN_DISTANCE{0.2f};     // m
	static constexpr float MAX_DISTANCE{40.0f};    // m, CAN3_40M
	static constexpr float AZIMUTH_FOV{112.0f};    // deg
	static constexpr float ELEVATION_FOV{14.0f};   // deg
	static constexpr hrt_abstime DATA_TIMEOUT{500_ms};

	int open_socket();
	void close_socket();
	void handle_frame(const struct can_frame &frame, hrt_abstime now);
	void start_cycle(const mr72_can::ObjectStatus &status, hrt_abstime now);
	void add_object(const mr72_can::Object &obj);
	void publish_cycle(bool complete);
	void record_raw_frame(const struct can_frame &frame);
	void print_controller_status() const;

	const int _bus;
	const uint8_t _sensor_id;
	const uint32_t _bitrate;
	const bool _raw;
	const bool _probe;
	uint32_t _device_id{0};

	int _sock{-1};
	bool _bitrate_set{false};

	// Identifiers seen in raw mode
	static constexpr int MAX_RAW_IDS{16};
	uint32_t _raw_ids[MAX_RAW_IDS] {};
	uint32_t _raw_counts[MAX_RAW_IDS] {};
	uint8_t _raw_last[MAX_RAW_IDS][8] {};
	uint8_t _raw_dlc[MAX_RAW_IDS] {};
	int _raw_id_count{0};
	uint32_t _raw_overflow{0};

	// Current measurement cycle
	bool _in_cycle{false};
	hrt_abstime _cycle_start{0};
	uint16_t _meas_count{0};
	uint8_t _expected{0};
	uint8_t _received{0};
	radar_objects_s _objects{};

	// Latest state and version reported by the radar
	bool _have_state{false};
	mr72_can::RadarState _state{};
	bool _have_version{false};
	mr72_can::Version _version{};

	hrt_abstime _last_frame{0};
	hrt_abstime _last_cycle{0};
	int32_t _sector_mask{7};
	float _last_nearest{-1.0f};

	PX4Rangefinder _px4_rangefinder;
	uORB::Publication<radar_objects_s> _radar_objects_pub{ORB_ID(radar_objects)};
	uORB::Publication<radar_status_s> _radar_status_pub{ORB_ID(radar_status)};

	perf_counter_t _frames_perf{perf_alloc(PC_COUNT, MODULE_NAME": frames")};
	perf_counter_t _cycles_perf{perf_alloc(PC_COUNT, MODULE_NAME": cycles")};
	perf_counter_t _incomplete_perf{perf_alloc(PC_COUNT, MODULE_NAME": incomplete cycles")};
	perf_counter_t _timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": data timeouts")};
	perf_counter_t _read_err_perf{perf_alloc(PC_COUNT, MODULE_NAME": read errors")};
};
