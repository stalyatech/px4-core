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

#include "FakeMR72.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

FakeMR72::FakeMR72() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_px4_rangefinder(0, distance_sensor_s::ROTATION_FORWARD_FACING)
{
	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_MR72;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_MR72);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_min_distance(MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MAX_DISTANCE);
	_px4_rangefinder.set_hfov(math::radians(AZIMUTH_FOV));
	_px4_rangefinder.set_vfov(math::radians(ELEVATION_FOV));

	_objects.device_id = device_id.devid;
}

bool FakeMR72::init()
{
	param_get(param_find("SENS_MR72_MASK"), &_sector_mask);
	ScheduleOnInterval(CYCLE_US);
	return true;
}

float FakeMR72::uniform()
{
	// xorshift32
	_rng ^= _rng << 13;
	_rng ^= _rng >> 17;
	_rng ^= _rng << 5;
	return ((_rng >> 8) + 1) * (1.f / 16777217.f);
}

float FakeMR72::gauss()
{
	const float u = uniform();
	const float v = uniform();
	return sqrtf(-2.f * logf(u)) * cosf(2.f * M_PI_F * v);
}

int FakeMR72::targets(const rig_sensor_pose_s &pose, Target out[2])
{
	const float half_elevation = math::radians(ELEVATION_FOV * 0.5f);
	const float noise = _param_fake_mr72_noise.get();
	int n = 0;

	// Corner reflector ahead of the rig: seen while its depression angle
	// lies inside the elevation beam.
	const float rx = _param_fake_mr72_rx.get() - pose.mr72_face_x;

	if (rx > 0.f) {
		const float dz = pose.mr72_face_z - _param_fake_mr72_rz.get();
		const float depression = atan2f(dz, rx);

		if (fabsf(depression - pose.mr72_boresight) <= half_elevation) {
			out[n++] = Target{ID_REFLECTOR, sqrtf(rx * rx + dz * dz) + noise * gauss(), 10.f};
		}
	}

	// Ground under the lower beam edge: H / sin(theta + beta).
	const float lower_edge = pose.mr72_boresight + half_elevation;

	if (lower_edge > math::radians(0.2f) && pose.mr72_face_z > 0.f && uniform() < GROUND_DETECTION) {
		const float range = pose.mr72_face_z / sinf(lower_edge);

		if (range <= MAX_DISTANCE) {
			out[n++] = Target{ID_GROUND, range + noise * gauss(), -10.f};
		}
	}

	// The radar lists objects nearest first.
	if (n == 2 && out[1].range < out[0].range) {
		const Target t = out[0];
		out[0] = out[1];
		out[1] = t;
	}

	return n;
}

void FakeMR72::publish_cycle(const Target targets[], int count, hrt_abstime now)
{
	const float dt = (_last_cycle > 0) ? (now - _last_cycle) * 1e-6f : 0.f;
	float seen[2] {-1.f, -1.f};
	uint8_t valid = 0;

	for (int i = 0; i < count; i++) {
		// Quantised like the CAN object list
		const float range = roundf(targets[i].range / DIST_RESOLUTION) * DIST_RESOLUTION;

		if (range < MIN_DISTANCE || range > MAX_DISTANCE) {
			continue;
		}

		const int slot = targets[i].id - 1;
		float vrel = 0.f;

		if (dt > 0.f && _last_range[slot] > 0.f) {
			vrel = roundf((range - _last_range[slot]) / dt / VREL_RESOLUTION) * VREL_RESOLUTION;
		}

		seen[slot] = range;

		_objects.id[valid] = targets[i].id;
		_objects.dist_long[valid] = range;
		_objects.dist_lat[valid] = roundf(_param_fake_mr72_noise.get() * gauss() / DIST_RESOLUTION) * DIST_RESOLUTION;
		_objects.vrel_long[valid] = vrel;
		_objects.vrel_lat[valid] = 0.f;
		_objects.rcs[valid] = roundf(targets[i].rcs / RCS_RESOLUTION) * RCS_RESOLUTION;
		_objects.sector[valid] = SECTOR_CENTRE;
		valid++;
	}

	_last_range[0] = seen[0];
	_last_range[1] = seen[1];
	_last_cycle = now;

	_objects.timestamp_sample = now;
	_objects.meas_count++;
	_objects.reported = valid;
	_objects.count = valid;
	_objects.timestamp = hrt_absolute_time();
	_radar_objects_pub.publish(_objects);

	// Nearest target per sector and inside SENS_MR72_MASK, as the driver does
	float sector_nearest[3] {-1.f, -1.f, -1.f};
	float nearest = -1.f;

	for (uint8_t i = 0; i < valid; i++) {
		const float d = _objects.dist_long[i];
		const uint8_t s = _objects.sector[i];

		if (sector_nearest[s - 1] < 0.f || d < sector_nearest[s - 1]) {
			sector_nearest[s - 1] = d;
		}

		if ((_sector_mask & (1 << (s - 1))) && (nearest < 0.f || d < nearest)) {
			nearest = d;
		}
	}

	radar_status_s status{};
	status.timestamp = _objects.timestamp;

	for (int i = 0; i < 3; i++) {
		status.sector[i] = sector_nearest[i];
	}

	for (int i = 0; i < 5; i++) {
		status.distance[i] = -1.f;
	}

	_radar_status_pub.publish(status);

	if (nearest > 0.f) {
		_px4_rangefinder.update(now, nearest, 100);

	} else {
		// Nothing in range: report the far limit with zero quality.
		_px4_rangefinder.update(now, MAX_DISTANCE, 0);
	}

	_last_nearest = nearest;
}

void FakeMR72::Run()
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
		param_get(param_find("SENS_MR72_MASK"), &_sector_mask);
	}

	rig_sensor_pose_s pose;

	if (!_rig_sensor_pose_sub.copy(&pose) || hrt_elapsed_time(&pose.timestamp) > 200_ms) {
		return;
	}

	Target found[2];
	const int count = targets(pose, found);
	publish_cycle(found, count, hrt_absolute_time());
}

int FakeMR72::print_status()
{
	PX4_INFO("cycle %u, %u objects, nearest %.1f m", _objects.meas_count, _objects.count, (double)_last_nearest);

	for (uint8_t i = 0; i < _objects.count; i++) {
		PX4_INFO("  id %u: %.1f m, vrel %.2f m/s, rcs %.1f", _objects.id[i], (double)_objects.dist_long[i],
			 (double)_objects.vrel_long[i], (double)_objects.rcs[i]);
	}

	return 0;
}

int FakeMR72::task_spawn(int argc, char *argv[])
{
	FakeMR72 *instance = new FakeMR72();

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

int FakeMR72::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeMR72::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simulated Nanoradar MR72 for the tilt rig. Takes the MR72 pose from rig_pose
and every 50 ms lists the targets inside its 14 degree elevation beam: a
corner reflector ahead of the rig (FAKE_MR72_RX, FAKE_MR72_RZ) and the ground
under the lower beam edge, H / sin(theta + 7 deg). Objects use the CAN
driver's 0.2 m distance and 0.25 m/s velocity units and go out as
radar_objects, radar_status and the nearest one as distance_sensor.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_mr72", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_mr72_main(int argc, char *argv[])
{
	return FakeMR72::main(argc, argv);
}
