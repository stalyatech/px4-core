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

#include "RigPose.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/parameters/param.h>

RigPose::RigPose() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool RigPose::init()
{
	param_get(param_find("MNT_RANGE_PITCH"), &_gimbal_range_pitch);
	ScheduleOnInterval(INTERVAL_US);
	return true;
}

bool RigPose::tilt(float dt, float &tilt_rad)
{
	switch (_param_rig_tilt_src.get()) {
	case rig_sensor_pose_s::SOURCE_IMU: {
			vehicle_attitude_s att;

			if (!_vehicle_attitude_sub.copy(&att) || hrt_elapsed_time(&att.timestamp) > 500_ms) {
				return false;
			}

			// The FMU sits on the cradle with x forward, so its pitch (nose up
			// positive) is the negative platform tilt.
			const matrix::Eulerf euler{matrix::Quatf(att.q)};
			tilt_rad = -euler.theta() - math::radians(_param_rig_tilt_off.get());
			return true;
		}

	case rig_sensor_pose_s::SOURCE_SERVO: {
			gimbal_controls_s controls;

			if (_gimbal_controls_sub.update(&controls)) {
				// gimbal_controls is normalised to half of MNT_RANGE_PITCH, nose up positive.
				_servo_cmd = -controls.control[gimbal_controls_s::INDEX_PITCH]
					     * math::radians(_gimbal_range_pitch * 0.5f);
			}

			// The servo follows its command as a first order lag.
			const float tau = _param_rig_servo_tau.get();
			const float alpha = (tau > 0.f) ? 1.f - expf(-dt / tau) : 1.f;
			_servo_tilt += (_servo_cmd - _servo_tilt) * alpha;
			tilt_rad = _servo_tilt;
			return true;
		}

	case rig_sensor_pose_s::SOURCE_MANUAL:
		tilt_rad = math::radians(_param_rig_tilt_deg.get());
		return true;

	default:
		return false;
	}
}

void RigPose::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		param_get(param_find("MNT_RANGE_PITCH"), &_gimbal_range_pitch);
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = (_last_run > 0) ? math::constrain((now - _last_run) * 1e-6f, 1e-3f, 0.1f) : 0.02f;
	_last_run = now;

	float theta = 0.f;

	if (!tilt(dt, theta)) {
		return;
	}

	const float c = cosf(theta);
	const float s = sinf(theta);
	const float h = _param_rig_axis_h.get();

	// Cradle points rotate about the axis; nose down moves +x points down.
	_pose.timestamp = now;
	_pose.source = static_cast<uint8_t>(_param_rig_tilt_src.get());
	_pose.tilt = theta;
	_pose.axis_height = h;
	_pose.alt_face_x = ALT_FACE_X * c + ALT_FACE_Z * s;
	_pose.alt_face_z = h - ALT_FACE_X * s + ALT_FACE_Z * c;
	_pose.alt_boresight = theta;
	_pose.mr72_face_x = MR72_FACE_X * c + MR72_FACE_Z * s;
	_pose.mr72_face_z = h - MR72_FACE_X * s + MR72_FACE_Z * c;
	_pose.mr72_boresight = theta;
	_rig_sensor_pose_pub.publish(_pose);
}

int RigPose::print_status()
{
	static constexpr const char *sources[] {"IMU", "servo", "manual"};
	const int src = _param_rig_tilt_src.get();
	PX4_INFO("source %s, tilt %.1f deg, H %.3f m", (src >= 0 && src <= 2) ? sources[src] : "?",
		 (double)math::degrees(_pose.tilt), (double)_pose.axis_height);
	PX4_INFO("altimeter face (%.3f, %.3f) m, MR72 face (%.3f, %.3f) m",
		 (double)_pose.alt_face_x, (double)_pose.alt_face_z, (double)_pose.mr72_face_x, (double)_pose.mr72_face_z);
	return 0;
}

int RigPose::task_spawn(int argc, char *argv[])
{
	RigPose *instance = new RigPose();

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

int RigPose::custom_command(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[0], "set")) {
		const float deg = strtof(argv[1], nullptr);
		const int32_t src = rig_sensor_pose_s::SOURCE_MANUAL;
		param_set(param_find("RIG_TILT_DEG"), &deg);
		param_set(param_find("RIG_TILT_SRC"), &src);
		return 0;
	}

	return print_usage("unknown command");
}

int RigPose::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Converts the tilt of the radar/IMU rig platform into the pose of every radar
on it and publishes rig_sensor_pose at 50 Hz. The tilt comes from the IMU
(vehicle_attitude), from the gimbal pitch command through a servo lag model,
or from a manual value (RIG_TILT_SRC). Positive tilt is nose down.

### Examples
Hold the platform at 20 degrees nose down for the radar simulators:
$ rig_pose set 20
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rig_pose", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Switch to the manual source with this tilt");
	PRINT_MODULE_USAGE_ARG("<deg>", "Tilt in degrees, positive nose down", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rig_pose_main(int argc, char *argv[])
{
	return RigPose::main(argc, argv);
}
