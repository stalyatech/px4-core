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

#include "RadarImuFusion.hpp"

#include "../rig_pose/rig_geometry.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

RadarImuFusion::RadarImuFusion() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_out.ground_range = -1.f;
	_out.nearest_obstacle = -1.f;
	_out.nees = NAN;
}

bool RadarImuFusion::init()
{
	update_parameters();

	if (!_sensor_combined_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void RadarImuFusion::update_parameters()
{
	updateParams();

	TiltHeightEkf::Config &cfg = _ekf.config();
	cfg.gyro_noise = _param_fus_gyr_noise.get();
	cfg.bias_random_walk = _param_fus_gbias_rw.get();
	cfg.height_random_walk = _param_fus_h_rw.get();
	cfg.alt_noise = _param_fus_alt_noise.get();
	cfg.ground_noise = _param_fus_gnd_noise.get();
	cfg.alt_gate = _param_fus_alt_gate.get();
	cfg.ground_gate = _param_fus_gnd_gate.get();
	cfg.half_beam = math::radians(rig::MR72_HALF_ELEVATION_DEG);
	cfg.alt_face_x = rig::ALT_FACE_X;
	cfg.alt_face_z = rig::ALT_FACE_Z;
	cfg.mr72_face_x = rig::MR72_FACE_X;
	cfg.mr72_face_z = rig::MR72_FACE_Z;
}

float RadarImuFusion::gauss()
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

bool RadarImuFusion::imu_sample(const sensor_combined_s &imu, float &rate, float &angle, float &noise, bool &angle_valid)
{
	noise = _param_fus_acc_noise.get();

	if (_param_fus_sim_imu.get()) {
		// Bench mode: build the IMU from the tilt rig_pose reports, with the
		// noise the filter assumes and a constant gyro bias.
		rig_sensor_pose_s pose;

		if (_rig_sensor_pose_sub.update(&pose)) {
			if (_sim_last_pose > 0) {
				const float pose_dt = (pose.timestamp - _sim_last_pose) * 1e-6f;

				if (pose_dt > 0.f) {
					_sim_rate = (pose.tilt - _sim_last_tilt) / pose_dt;
					_sim_start_tilt = _sim_last_tilt;
				}
			}

			_sim_last_tilt = pose.tilt;
			_sim_last_pose = pose.timestamp;
			_pose = pose;
		}

		if (_sim_last_pose == 0 || hrt_elapsed_time(&_sim_last_pose) > 200_ms) {
			return false;
		}

		// rig_pose runs at 50 Hz and the rate is a backward difference held for
		// one interval, so this is the trajectory the simulated gyro integrates:
		// one pose sample behind rig_pose. Use it as the truth as well.
		_sim_tilt = _sim_start_tilt + _sim_rate * hrt_elapsed_time(&_sim_last_pose) * 1e-6f;

		const float dt = math::max(imu.gyro_integral_dt * 1e-6f, 1e-4f);
		rate = _sim_rate + _param_fus_sim_bias.get() + _param_fus_gyr_noise.get() / sqrtf(dt) * gauss();
		angle = _sim_tilt + noise * gauss();
		angle_valid = true;
		return true;
	}

	// The FMU sits on the cradle with x forward: nose up is a positive y rate
	// and gravity gives pitch = atan2(ax, -az).
	const float ax = imu.accelerometer_m_s2[0];
	const float ay = imu.accelerometer_m_s2[1];
	const float az = imu.accelerometer_m_s2[2];
	const float norm = sqrtf(ax * ax + ay * ay + az * az);

	rate = -imu.gyro_rad[1];
	angle = atan2f(-ax, -az) - math::radians(_param_rig_tilt_off.get());
	angle_valid = fabsf(norm - CONSTANTS_ONE_G) < GRAVITY_TOLERANCE;
	return true;
}

void RadarImuFusion::update_altimeter()
{
	for (int i = 0; i < _distance_sensor_subs.size(); i++) {
		distance_sensor_s ds;

		if (!_distance_sensor_subs[i].update(&ds)) {
			continue;
		}

		if (ds.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING || ds.signal_quality == 0
		    || ds.current_distance < ds.min_distance || ds.current_distance > ds.max_distance) {
			continue;
		}

		const TiltHeightEkf::Innovation innovation = _ekf.update_altimeter(ds.current_distance);
		_out.alt_innovation = innovation.innovation;
		_out.alt_nis = innovation.nis;

		if (innovation.accepted) {
			_alt_rejects_in_a_row = 0;

		} else {
			_out.alt_rejected++;

			if (++_alt_rejects_in_a_row >= ALT_REJECTS_BEFORE_RESET) {
				// The height really changed (a spacer was added, for example).
				_ekf.reset_height(_ekf.height_from_range(ds.current_distance));
				_alt_rejects_in_a_row = 0;
			}
		}
	}
}

void RadarImuFusion::update_obstacle_radar()
{
	radar_objects_s objects;

	if (!_radar_objects_sub.update(&objects)) {
		return;
	}

	uint8_t ground = 0;
	uint8_t obstacles = 0;
	uint16_t mask = 0;
	float nearest = -1.f;
	float best_range = -1.f;
	float best_nis = INFINITY;

	for (uint8_t i = 0; i < objects.count && i < radar_objects_s::MAX_OBJECTS; i++) {
		const float range = sqrtf(objects.dist_long[i] * objects.dist_long[i] + objects.dist_lat[i] * objects.dist_lat[i]);
		const TiltHeightEkf::Innovation check = _ekf.check_ground(range);

		if (check.accepted) {
			ground++;
			mask |= (1 << i);

			if (check.nis < best_nis) {
				best_nis = check.nis;
				best_range = range;
			}

		} else {
			obstacles++;

			if (nearest < 0.f || range < nearest) {
				nearest = range;
			}
		}
	}

	_out.ground_count = ground;
	_out.obstacle_count = obstacles;
	_out.ground_mask = mask;
	_out.nearest_obstacle = nearest;

	// Off by default: a real target inside the gate would bias the tilt.
	if (_param_fus_gnd_upd.get() && best_range > 0.f) {
		_ekf.update_ground(best_range);
	}
}

void RadarImuFusion::publish(hrt_abstime now)
{
	_out.tilt = _ekf.tilt();
	_out.tilt_std = sqrtf(_ekf.tilt_variance());
	_out.gyro_bias = _ekf.bias();
	_out.height = _ekf.height();
	_out.height_std = sqrtf(_ekf.height_variance());

	const bool ground_visible = _ekf.ground_visible();
	_out.ground_range = ground_visible ? _ekf.ground_range() : -1.f;
	_out.ground_gate = ground_visible ? _ekf.ground_gate_width() : 0.f;

	_out.nees = _param_fus_sim_imu.get() ? _ekf.nees(_sim_tilt, _param_fus_sim_bias.get(), _pose.axis_height) : NAN;

	_out.timestamp = now;
	_radar_fusion_pub.publish(_out);

	// The rig UI reads this as DEBUG_FLOAT_ARRAY "fusion"
	debug_array_s dbg{};
	dbg.timestamp = now;
	dbg.id = 1;
	strncpy(dbg.name, "fusion", sizeof(dbg.name));
	dbg.data[0] = math::degrees(_out.tilt);
	dbg.data[1] = math::degrees(_out.tilt_std);
	dbg.data[2] = _out.height;
	dbg.data[3] = _out.height_std;
	dbg.data[4] = _out.ground_range;
	dbg.data[5] = _out.ground_gate;
	dbg.data[6] = _out.nearest_obstacle;
	dbg.data[7] = _out.ground_count;
	dbg.data[8] = _out.obstacle_count;
	dbg.data[9] = _out.alt_nis;
	dbg.data[10] = _out.nees;
	dbg.data[11] = math::degrees(_out.gyro_bias);
	_debug_array_pub.publish(dbg);
}

void RadarImuFusion::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		update_parameters();
	}

	sensor_combined_s imu;

	if (!_sensor_combined_sub.update(&imu) || imu.gyro_integral_dt == 0) {
		return;
	}

	float rate = 0.f;
	float angle = 0.f;
	float noise = 0.f;
	bool angle_valid = false;

	if (!imu_sample(imu, rate, angle, noise, angle_valid)) {
		return;
	}

	if (!_initialised) {
		if (!angle_valid) {
			return;
		}

		_ekf.init(angle, _param_rig_axis_h.get());
		_initialised = true;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Layer 1: gyro prediction, gravity direction as tilt
	_ekf.predict(rate, math::constrain(imu.gyro_integral_dt * 1e-6f, 1e-4f, 0.05f));

	if (angle_valid && now - _last_tilt_update >= TILT_UPDATE_INTERVAL) {
		_ekf.update_tilt(angle, noise);
		_last_tilt_update = now;
	}

	// Layer 2: altimeter
	update_altimeter();

	// Layer 3: ground echoes of the obstacle radar
	update_obstacle_radar();

	if (now - _last_publish >= PUBLISH_INTERVAL) {
		publish(now);
		_last_publish = now;
	}
}

int RadarImuFusion::print_status()
{
	PX4_INFO("%s IMU, %s", _param_fus_sim_imu.get() ? "simulated" : "real", _initialised ? "running" : "waiting for IMU");
	PX4_INFO("tilt %.2f +- %.2f deg, bias %.3f deg/s", (double)math::degrees(_out.tilt),
		 (double)math::degrees(_out.tilt_std), (double)math::degrees(_out.gyro_bias));
	PX4_INFO("height %.4f +- %.4f m, altimeter NIS %.2f, rejected %" PRIu32, (double)_out.height, (double)_out.height_std,
		 (double)_out.alt_nis, _out.alt_rejected);
	PX4_INFO("ground echo %.2f +- %.2f m, last cycle %u ground, %u obstacles, nearest %.2f m", (double)_out.ground_range,
		 (double)_out.ground_gate, _out.ground_count, _out.obstacle_count, (double)_out.nearest_obstacle);

	if (PX4_ISFINITE(_out.nees)) {
		PX4_INFO("NEES %.2f", (double)_out.nees);
	}

	return 0;
}

int RadarImuFusion::task_spawn(int argc, char *argv[])
{
	RadarImuFusion *instance = new RadarImuFusion();

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

int RadarImuFusion::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RadarImuFusion::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Radar/IMU fusion for the tilt rig. One extended Kalman filter with the state
tilt, gyro bias and axis height runs three layers: gyro and accelerometer give
the tilt, the altimeter slant range gives the height with the tilt uncertainty
taken into account, and the expected ground echo of the obstacle radar with a
gate scaled by the estimate uncertainty separates ground echoes from real
obstacles. Results go out as radar_fusion and as debug array "fusion" for the
rig UI. FUS_SIM_IMU builds the IMU from rig_pose for benches where the FMU does
not tilt, and then also reports NEES against the known truth.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("radar_imu_fusion", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int radar_imu_fusion_main(int argc, char *argv[])
{
	return RadarImuFusion::main(argc, argv);
}
