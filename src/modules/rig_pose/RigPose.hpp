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
 * @file RigPose.hpp
 *
 * Converts the tilt of the radar/IMU rig platform into the poses of the
 * radars mounted on it (rig_sensor_pose). The radar simulators and later the
 * fusion estimator take the sensor geometry from here.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/gimbal_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rig_sensor_pose.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace time_literals;

class RigPose : public ModuleBase<RigPose>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RigPose();
	~RigPose() override = default;

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
	static constexpr hrt_abstime INTERVAL_US{20_ms}; // 50 Hz

	void Run() override;

	/** Platform tilt [rad] from the selected source, positive nose down. */
	bool tilt(float dt, float &tilt_rad);

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _gimbal_controls_sub{ORB_ID(gimbal_controls)};

	uORB::Publication<rig_sensor_pose_s> _rig_sensor_pose_pub{ORB_ID(rig_sensor_pose)};

	rig_sensor_pose_s _pose{};
	float _servo_tilt{0.f};
	float _servo_cmd{0.f};
	float _gimbal_range_pitch{90.f};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RIG_TILT_SRC>) _param_rig_tilt_src,
		(ParamFloat<px4::params::RIG_TILT_DEG>) _param_rig_tilt_deg,
		(ParamFloat<px4::params::RIG_TILT_OFF>) _param_rig_tilt_off,
		(ParamFloat<px4::params::RIG_AXIS_H>) _param_rig_axis_h,
		(ParamFloat<px4::params::RIG_SERVO_TAU>) _param_rig_servo_tau
	)
};
