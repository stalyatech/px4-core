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

#include "TiltRig.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

TiltRig::TiltRig() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool TiltRig::init()
{
	ScheduleOnInterval(INTERVAL_US);
	return true;
}

float TiltRig::command(float t) const
{
	const float a = _amplitude;
	const float p = _period;
	const float ph = fmodf(t, p) / p;

	switch (_kind) {
	case Kind::Step:
		return (ph < 0.5f) ? a : -a;

	case Kind::Ramp:
		// Triangle -A ... +A ... -A
		return a * ((ph < 0.5f) ? 4.f * ph - 1.f : 3.f - 4.f * ph);

	case Kind::Sine:
		return a * sinf(2.f * M_PI_F * ph);

	case Kind::Chirp: {
			// 0.05 to 1 Hz linear sweep over four periods
			const float T = 4.f * p;
			const float tc = fmodf(t, T);
			const float f0 = 0.05f;
			const float f1 = 1.f;
			return a * sinf(2.f * M_PI_F * (f0 * tc + (f1 - f0) * tc * tc / (2.f * T)));
		}

	case Kind::Stairs:
		// Five levels -A, -A/2, 0, +A/2, +A
		return a * (floorf(ph * 5.f) / 2.f - 1.f);

	default:
		return 0.f;
	}
}

void TiltRig::ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(command_ack);
}

void TiltRig::take_gimbal_control()
{
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	cmd.param1 = _param_mav_sys_id.get();  // primary control sysid
	cmd.param2 = COMPONENT_ID;             // primary control compid
	cmd.param3 = -1.f;                     // secondary control unchanged
	cmd.param4 = -1.f;
	cmd.source_system = _param_mav_sys_id.get();
	cmd.source_component = COMPONENT_ID;
	cmd.target_system = _param_mav_sys_id.get();
	cmd.target_component = 0;              // any gimbal manager
	cmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(cmd);
}

void TiltRig::handle_command(const vehicle_command_s &cmd)
{
	const int kind = roundf(cmd.param1);

	if (kind < static_cast<int>(Kind::Stop) || kind > static_cast<int>(Kind::Stairs)) {
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		mavlink_log_critical(&_mavlink_log_pub, "tilt_rig: unknown profile %d", kind);
		return;
	}

	ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

	if (kind == static_cast<int>(Kind::Stop)) {
		if (_running) {
			mavlink_log_info(&_mavlink_log_pub, "tilt_rig: stopped");
		}

		_running = false;
		_kind = Kind::Stop;
		_hold_zero = true;
		_start = hrt_absolute_time();
		return;
	}

	_kind = static_cast<Kind>(kind);
	_amplitude = math::constrain(fabsf(cmd.param2), 0.f, AMPLITUDE_MAX);
	_period = math::max(cmd.param3, PERIOD_MIN);
	_cycles = math::max(static_cast<int>(roundf(cmd.param4)), 0);
	_start = hrt_absolute_time();
	_running = true;
	_hold_zero = false;

	take_gimbal_control();

	static constexpr const char *names[] {"stop", "step", "ramp", "sine", "chirp", "stairs"};
	mavlink_log_info(&_mavlink_log_pub, "tilt_rig: %s +-%.0f deg, %.1f s, %d cycles", names[kind],
			 (double)_amplitude, (double)_period, _cycles);
}

void TiltRig::publish_setpoint(float tilt_deg)
{
	_tilt_cmd = tilt_deg;

	// Gimbal pitch is nose up positive, the rig tilt nose down positive.
	const matrix::Quatf q{matrix::Eulerf(0.f, math::radians(-tilt_deg), 0.f)};

	gimbal_manager_set_attitude_s setpoint{};
	setpoint.origin_sysid = _param_mav_sys_id.get();
	setpoint.origin_compid = COMPONENT_ID;
	setpoint.target_system = _param_mav_sys_id.get();
	setpoint.target_component = 0;
	setpoint.flags = 0;
	q.copyTo(setpoint.q);
	setpoint.angular_velocity_x = NAN;
	setpoint.angular_velocity_y = NAN;
	setpoint.angular_velocity_z = NAN;
	setpoint.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(setpoint);

	// Commanded tilt for the rig UI (NAMED_VALUE_FLOAT "tilt_cmd", degrees)
	debug_key_value_s dbg{};
	strncpy(dbg.key, "tilt_cmd", sizeof(dbg.key));
	dbg.value = tilt_deg;
	dbg.timestamp = setpoint.timestamp;
	_debug_key_value_pub.publish(dbg);
}

void TiltRig::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	vehicle_command_s cmd;

	while (_vehicle_command_sub.update(&cmd)) {
		if (cmd.command == CMD_TILT_PROFILE
		    && (cmd.target_system == _param_mav_sys_id.get() || cmd.target_system == 0)) {
			handle_command(cmd);
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	const float t = (now - _start) * 1e-6f;

	if (_running) {
		if (_cycles > 0 && t >= _cycles * cycle_length()) {
			_running = false;
			_hold_zero = true;
			_start = now;
			mavlink_log_info(&_mavlink_log_pub, "tilt_rig: profile done");

		} else {
			publish_setpoint(command(t));
			return;
		}
	}

	if (_hold_zero) {
		// Return to level and keep sending it briefly so the gimbal settles there.
		publish_setpoint(0.f);
		_hold_zero = (now - _start) < 1_s;
	}
}

int TiltRig::print_status()
{
	static constexpr const char *names[] {"stop", "step", "ramp", "sine", "chirp", "stairs"};
	PX4_INFO("%s, %s +-%.0f deg, %.1f s, %d cycles, command %.1f deg", _running ? "running" : "idle",
		 names[static_cast<int>(_kind)], (double)_amplitude, (double)_period, _cycles, (double)_tilt_cmd);
	return 0;
}

int TiltRig::task_spawn(int argc, char *argv[])
{
	TiltRig *instance = new TiltRig();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int TiltRig::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TiltRig::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Runs tilt profiles on the radar/IMU rig. MAV_CMD_USER_1 (31010) selects the
profile: param1 kind (0 stop, 1 step, 2 ramp, 3 sine, 4 chirp 0.05 to 1 Hz,
5 stairs), param2 amplitude in degrees (up to 60), param3 period in seconds,
param4 cycles (0 runs until stopped). The module takes primary control of the
gimbal manager and streams the commanded tilt at 50 Hz; positive tilt is nose
down. The command is published as debug value "tilt_cmd" for the rig UI.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tilt_rig", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int tilt_rig_main(int argc, char *argv[])
{
	return TiltRig::main(argc, argv);
}
