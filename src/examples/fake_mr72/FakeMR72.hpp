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
 * @file FakeMR72.hpp
 *
 * Simulated Nanoradar MR72 obstacle radar on the tilt rig. Reads the MR72
 * pose from rig_sensor_pose and publishes radar_objects, radar_status and
 * distance_sensor like the CAN driver.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/radar_objects.h>
#include <uORB/topics/radar_status.h>
#include <uORB/topics/rig_sensor_pose.h>

using namespace time_literals;

class FakeMR72 : public ModuleBase<FakeMR72>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FakeMR72();
	~FakeMR72() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	// Same characteristics as the CAN driver (NanoRadarMR72Can) and the
	// ARS408 style object list the MR72 sends
	static constexpr hrt_abstime CYCLE_US{50_ms};
	static constexpr float MIN_DISTANCE{0.2f};     // m
	static constexpr float MAX_DISTANCE{40.0f};    // m, CAN3_40M
	static constexpr float AZIMUTH_FOV{112.0f};    // deg
	static constexpr float ELEVATION_FOV{14.0f};   // deg
	static constexpr float DIST_RESOLUTION{0.2f};  // m, object distance unit
	static constexpr float VREL_RESOLUTION{0.25f}; // m/s, object velocity unit
	static constexpr float RCS_RESOLUTION{0.5f};   // dBm2

	// Ground echo of the lower beam edge is not seen in every cycle.
	static constexpr float GROUND_DETECTION{0.85f};

	static constexpr uint8_t ID_REFLECTOR{1};
	static constexpr uint8_t ID_GROUND{2};
	static constexpr uint8_t SECTOR_CENTRE{2};

	struct Target {
		uint8_t id;
		float range;
		float rcs;
	};

	void Run() override;

	/** Targets inside the beam this cycle, returns their number. */
	int targets(const rig_sensor_pose_s &pose, Target out[2]);

	void publish_cycle(const Target targets[], int count, hrt_abstime now);

	float uniform();
	float gauss();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _rig_sensor_pose_sub{ORB_ID(rig_sensor_pose)};

	uORB::Publication<radar_objects_s> _radar_objects_pub{ORB_ID(radar_objects)};
	uORB::Publication<radar_status_s> _radar_status_pub{ORB_ID(radar_status)};

	PX4Rangefinder _px4_rangefinder;

	radar_objects_s _objects{};
	float _last_range[2] {-1.f, -1.f};   // per target id, for the radial velocity
	hrt_abstime _last_cycle{0};
	int32_t _sector_mask{7};
	uint32_t _rng{0x2545f491};
	float _last_nearest{-1.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FAKE_MR72_RX>) _param_fake_mr72_rx,
		(ParamFloat<px4::params::FAKE_MR72_RZ>) _param_fake_mr72_rz,
		(ParamFloat<px4::params::FAKE_MR72_WALL>) _param_fake_mr72_wall,
		(ParamFloat<px4::params::FAKE_MR72_NOISE>) _param_fake_mr72_noise
	)
};
