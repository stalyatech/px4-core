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
 * @file FakeNRA15.hpp
 *
 * Simulated Nanoradar NRA15 altimeter on the tilt rig. Reads the altimeter
 * pose from rig_sensor_pose and publishes distance_sensor like the CAN driver.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rig_sensor_pose.h>

using namespace time_literals;

class FakeNRA15 : public ModuleBase<FakeNRA15>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FakeNRA15();
	~FakeNRA15() override = default;

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
	// Same characteristics as the CAN driver (NanoRadarNRA15Can)
	static constexpr hrt_abstime CYCLE_US{25_ms};
	static constexpr float MIN_DISTANCE{0.1f};     // m
	static constexpr float MAX_DISTANCE{100.0f};   // m
	static constexpr float FIELD_OF_VIEW{37.0f};   // deg
	static constexpr float RESOLUTION{0.01f};      // m, CAN range unit

	void Run() override;

	/** Range to the first ground return [m], negative without a return. */
	float ground_range(const rig_sensor_pose_s &pose) const;

	float gauss();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _rig_sensor_pose_sub{ORB_ID(rig_sensor_pose)};

	PX4Rangefinder _px4_rangefinder;

	uint32_t _rng{0x9e3779b9};
	float _last_range{-1.f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FAKE_NRA_BEAM>) _param_fake_nra_beam,
		(ParamFloat<px4::params::FAKE_NRA_NOISE>) _param_fake_nra_noise
	)
};
