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
 * @file TiltRig.hpp
 *
 * Runs tilt profiles on the radar/IMU rig. MAV_CMD_USER_1 (31010) from the
 * rig UI selects the profile; the module takes primary control of the gimbal
 * manager and streams the commanded pitch to it, which drives the tilt servo
 * on the TPU IO.
 */

#pragma once

#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

using namespace time_literals;

class TiltRig : public ModuleBase<TiltRig>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TiltRig();
	~TiltRig() override = default;

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

	// MAV_CMD_USER_1: p1 kind, p2 amplitude [deg], p3 period [s], p4 cycles (0 runs until stopped)
	static constexpr uint16_t CMD_TILT_PROFILE{31010};

	// Our identity towards the gimbal manager (MAV_COMP_ID_USER1)
	static constexpr uint8_t COMPONENT_ID{25};

	static constexpr float AMPLITUDE_MAX{60.f};    // deg, mechanical range of the cradle
	static constexpr float PERIOD_MIN{0.5f};       // s

	enum class Kind : uint8_t {
		Stop = 0,
		Step = 1,
		Ramp = 2,
		Sine = 3,
		Chirp = 4,
		Stairs = 5,
	};

	void Run() override;

	void handle_command(const vehicle_command_s &cmd);
	void ack(const vehicle_command_s &cmd, uint8_t result);
	void take_gimbal_control();

	/** Commanded tilt [deg] at time t [s] of the profile, positive nose down. */
	float command(float t) const;

	/** Length of one profile cycle [s]. */
	float cycle_length() const { return (_kind == Kind::Chirp) ? 4.f * _period : _period; }

	void publish_setpoint(float tilt_deg);

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	uORB::Publication<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<debug_key_value_s> _debug_key_value_pub{ORB_ID(debug_key_value)};

	orb_advert_t _mavlink_log_pub{nullptr};

	Kind _kind{Kind::Stop};
	float _amplitude{0.f};
	float _period{1.f};
	int _cycles{0};
	hrt_abstime _start{0};
	float _tilt_cmd{0.f};
	bool _running{false};
	bool _hold_zero{false};     // send 0 deg a little longer after a stop

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id
	)
};
