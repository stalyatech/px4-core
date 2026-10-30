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
 * @file RadarImuFusion.hpp
 *
 * Radar/IMU fusion on the tilt rig: estimates the platform tilt and the axis
 * height with TiltHeightEkf and separates the ground echoes of the obstacle
 * radar from real obstacles.
 */

#pragma once

#include "TiltHeightEkf.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/radar_fusion.h>
#include <uORB/topics/radar_objects.h>
#include <uORB/topics/rig_sensor_pose.h>
#include <uORB/topics/sensor_combined.h>

using namespace time_literals;

class RadarImuFusion : public ModuleBase<RadarImuFusion>, public ModuleParams, public px4::WorkItem
{
public:
	RadarImuFusion();
	~RadarImuFusion() override = default;

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
	static constexpr hrt_abstime TILT_UPDATE_INTERVAL{20_ms};  // 50 Hz
	static constexpr hrt_abstime PUBLISH_INTERVAL{40_ms};      // 25 Hz
	static constexpr float GRAVITY_TOLERANCE{2.f};             // [m/s^2] accel not trusted as tilt beyond this
	static constexpr int ALT_REJECTS_BEFORE_RESET{40};         // 1 s of altimeter ranges

	void Run() override;

	void update_parameters();

	/** Nose down rate [rad/s] and tilt from gravity [rad] of this sensor sample. */
	bool imu_sample(const sensor_combined_s &imu, float &rate, float &angle, float &noise, bool &angle_valid);

	void update_altimeter();
	void update_obstacle_radar();
	void publish(hrt_abstime now);

	float gauss();

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::Subscription _radar_objects_sub{ORB_ID(radar_objects)};
	uORB::Subscription _rig_sensor_pose_sub{ORB_ID(rig_sensor_pose)};

	uORB::Publication<radar_fusion_s> _radar_fusion_pub{ORB_ID(radar_fusion)};
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};

	TiltHeightEkf _ekf{};
	bool _initialised{false};

	radar_fusion_s _out{};
	hrt_abstime _last_tilt_update{0};
	hrt_abstime _last_publish{0};
	int _alt_rejects_in_a_row{0};

	// Simulated IMU
	rig_sensor_pose_s _pose{};
	float _sim_tilt{0.f};
	float _sim_start_tilt{0.f};
	float _sim_last_tilt{0.f};
	hrt_abstime _sim_last_pose{0};
	float _sim_rate{0.f};
	uint32_t _rng{0x6c078965};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FUS_SIM_IMU>) _param_fus_sim_imu,
		(ParamFloat<px4::params::FUS_SIM_BIAS>) _param_fus_sim_bias,
		(ParamFloat<px4::params::FUS_GYR_NOISE>) _param_fus_gyr_noise,
		(ParamFloat<px4::params::FUS_GBIAS_RW>) _param_fus_gbias_rw,
		(ParamFloat<px4::params::FUS_H_RW>) _param_fus_h_rw,
		(ParamFloat<px4::params::FUS_ACC_NOISE>) _param_fus_acc_noise,
		(ParamFloat<px4::params::FUS_ALT_NOISE>) _param_fus_alt_noise,
		(ParamFloat<px4::params::FUS_ALT_GATE>) _param_fus_alt_gate,
		(ParamFloat<px4::params::FUS_GND_NOISE>) _param_fus_gnd_noise,
		(ParamFloat<px4::params::FUS_GND_GATE>) _param_fus_gnd_gate,
		(ParamBool<px4::params::FUS_GND_UPD>) _param_fus_gnd_upd,
		(ParamFloat<px4::params::RIG_AXIS_H>) _param_rig_axis_h,
		(ParamFloat<px4::params::RIG_TILT_OFF>) _param_rig_tilt_off
	)
};
