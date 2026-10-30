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

#include "FakeNRA15.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>

FakeNRA15::FakeNRA15() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_px4_rangefinder(0, distance_sensor_s::ROTATION_DOWNWARD_FACING)
{
	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_NRA15;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_NRA15);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_min_distance(MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MAX_DISTANCE);
	_px4_rangefinder.set_fov(math::radians(FIELD_OF_VIEW));
}

bool FakeNRA15::init()
{
	ScheduleOnInterval(CYCLE_US);
	return true;
}

float FakeNRA15::gauss()
{
	// xorshift32 and Box-Muller
	auto uniform = [this]() {
		_rng ^= _rng << 13;
		_rng ^= _rng >> 17;
		_rng ^= _rng << 5;
		return ((_rng >> 8) + 1) * (1.f / 16777217.f);
	};

	const float u = uniform();
	const float v = uniform();
	return sqrtf(-2.f * logf(u)) * cosf(2.f * M_PI_F * v);
}

float FakeNRA15::ground_range(const rig_sensor_pose_s &pose) const
{
	if (pose.alt_face_z <= 0.f) {
		return -1.f;
	}

	float angle = fabsf(pose.alt_boresight);

	if (_param_fake_nra_beam.get() == 1) {
		// Flat ground returns strongest at normal incidence: while the vertical
		// lies inside the beam the radar sees the face height, beyond that the
		// beam edge nearest to the vertical.
		angle = math::max(angle - math::radians(FIELD_OF_VIEW * 0.5f), 0.f);
	}

	if (angle >= math::radians(80.f)) {
		return -1.f;
	}

	// Along the boresight from the face; for a face on the axis line this is
	// the proposal's R = H / cos(theta) - d.
	return pose.alt_face_z / cosf(angle);
}

void FakeNRA15::Run()
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
	}

	rig_sensor_pose_s pose;

	if (!_rig_sensor_pose_sub.copy(&pose) || hrt_elapsed_time(&pose.timestamp) > 200_ms) {
		// No pose, no radar cycle (the real driver also goes quiet without frames).
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	float range = ground_range(pose);

	if (range > 0.f) {
		range += _param_fake_nra_noise.get() * gauss();
		range = roundf(range / RESOLUTION) * RESOLUTION;
	}

	if (range >= MIN_DISTANCE && range <= MAX_DISTANCE) {
		_px4_rangefinder.update(now, range, 100);
		_last_range = range;

	} else {
		// Nothing detected: report the far limit with zero quality.
		_px4_rangefinder.update(now, MAX_DISTANCE, 0);
		_last_range = -1.f;
	}
}

int FakeNRA15::print_status()
{
	PX4_INFO("%s model, last range %.2f m", _param_fake_nra_beam.get() == 1 ? "beam edge" : "narrow beam",
		 (double)_last_range);
	return 0;
}

int FakeNRA15::task_spawn(int argc, char *argv[])
{
	FakeNRA15 *instance = new FakeNRA15();

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

int FakeNRA15::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeNRA15::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simulated Nanoradar NRA15 altimeter for the tilt rig. Takes the altimeter pose
from rig_pose and publishes distance_sensor every 25 ms with the CAN driver's
range limits, 1 cm resolution and downward orientation, so the MAVLink and UI
path can be tested without the radar. FAKE_NRA_BEAM selects the narrow beam
model of the proposal or the nearest return inside the 37 degree beam.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_nra15", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_nra15_main(int argc, char *argv[])
{
	return FakeNRA15::main(argc, argv);
}
