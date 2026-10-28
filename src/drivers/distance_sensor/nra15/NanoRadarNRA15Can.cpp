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

#include "NanoRadarNRA15Can.hpp"
#include "../nanoradar_socketcan.hpp"

#include <poll.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

using namespace nra15_can;

NanoRadarNRA15Can::NanoRadarNRA15Can(int bus, uint8_t rotation) :
	_bus(bus),
	_px4_rangefinder(0, rotation)
{
	device::Device::DeviceId device_id{};
	// PX4 has no plain CAN bus type; the UAVCAN one stands for "on a CAN bus".
	device_id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;
	device_id.devid_s.bus = bus;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_NRA15;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_NRA15);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_min_distance(MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MAX_DISTANCE);
	_px4_rangefinder.set_fov(math::radians(FIELD_OF_VIEW));
}

NanoRadarNRA15Can::~NanoRadarNRA15Can()
{
	if (_sock >= 0) {
		close(_sock);
	}

	perf_free(_frames_perf);
	perf_free(_cycles_perf);
	perf_free(_missed_perf);
	perf_free(_incomplete_perf);
	perf_free(_timeout_perf);
	perf_free(_read_err_perf);
}

void NanoRadarNRA15Can::run()
{
	const struct can_filter filters[] = {
		nanoradar_can::standard_id_filter(ID_SENSOR_BACK),
		nanoradar_can::standard_id_filter(ID_TARGET_STATUS),
		nanoradar_can::standard_id_filter(ID_TARGET_INFO),
	};

	_sock = nanoradar_can::open_socket(_bus, BITRATE, filters, sizeof(filters) / sizeof(filters[0]), _bitrate_set);

	if (_sock < 0) {
		return;
	}

	struct pollfd fds {};
	fds.fd = _sock;
	fds.events = POLLIN;

	bool timed_out = false;

	while (!should_exit()) {
		const int ret = poll(&fds, 1, 50);
		const hrt_abstime now = hrt_absolute_time();

		if (ret > 0 && (fds.revents & POLLIN)) {
			struct can_frame frame;
			const ssize_t n = read(_sock, &frame, sizeof(frame));

			if (n == sizeof(frame)) {
				handle_frame(frame, now);

			} else {
				perf_count(_read_err_perf);
			}

		} else if (ret < 0 && errno != EINTR) {
			perf_count(_read_err_perf);
			px4_usleep(10_ms);
		}

		// Ask for the version until the radar answers
		if (!_have_version && now - _version_requested > VERSION_RETRY) {
			request_version();
			_version_requested = now;
		}

		// Count each outage once; distance_sensor simply goes stale.
		if (_last_cycle != 0 && now - _last_cycle > DATA_TIMEOUT) {
			if (!timed_out) {
				perf_count(_timeout_perf);
				timed_out = true;
			}

		} else {
			timed_out = false;
		}
	}

	close(_sock);
	_sock = -1;
}

void NanoRadarNRA15Can::request_version()
{
	struct can_frame frame {};
	frame.can_id = ID_SENSOR_CONFIG;
	frame.can_dlc = 8;
	make_version_request(frame.data);

	// A full TX queue (radar not powered yet) is not an error worth a message
	write(_sock, &frame, sizeof(frame));
}

void NanoRadarNRA15Can::handle_frame(const struct can_frame &frame, hrt_abstime now)
{
	_last_frame = now;
	perf_count(_frames_perf);

	if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) {
		return;
	}

	const uint32_t id = frame.can_id & CAN_SFF_MASK;

	if (id == ID_TARGET_STATUS) {
		TargetStatus status{};

		if (parse_target_status(frame.data, frame.can_dlc, status)) {
			start_cycle(status, now);
		}

	} else if (id == ID_TARGET_INFO) {
		TargetInfo target{};

		if (_in_cycle && parse_target_info(frame.data, frame.can_dlc, target)) {
			add_target(target);

			if (_received >= _expected) {
				publish_cycle(true);
			}
		}

	} else if (id == ID_SENSOR_BACK) {
		SensorBack back{};

		if (parse_sensor_back(frame.data, frame.can_dlc, back) && back.data_type == DATA_TYPE_VERSION && back.ok) {
			memcpy(_version, back.param, sizeof(_version));
			_have_version = true;
		}
	}
}

void NanoRadarNRA15Can::start_cycle(const TargetStatus &status, hrt_abstime now)
{
	if (_in_cycle) {
		// A new cycle before all targets of the previous one arrived
		perf_count(_incomplete_perf);
		publish_cycle(false);
	}

	// The roll count steps once per cycle: a jump means lost cycles
	if (_have_roll_count && status.roll_count != next_roll_count(_roll_count)) {
		perf_count(_missed_perf);
	}

	_roll_count = status.roll_count;
	_have_roll_count = true;

	_in_cycle = true;
	_cycle_start = now;
	_expected = status.num_targets;
	_received = 0;
	_nearest = -1.0f;
	_num_targets = 0;

	if (_expected == 0) {
		publish_cycle(true);
	}
}

void NanoRadarNRA15Can::add_target(const TargetInfo &target)
{
	_received++;

	if (_num_targets < MAX_TARGETS) {
		_targets[_num_targets++] = target;
	}

	if (target.range >= MIN_DISTANCE && (_nearest < 0.0f || target.range < _nearest)) {
		_nearest = target.range;
	}
}

void NanoRadarNRA15Can::publish_cycle(bool complete)
{
	if (_nearest > 0.0f) {
		_px4_rangefinder.update(_cycle_start, _nearest, 100);

	} else {
		// Nothing detected: report the far limit with zero quality.
		_px4_rangefinder.update(_cycle_start, MAX_DISTANCE, 0);
	}

	_last_nearest = _nearest;
	_last_cycle = _cycle_start;
	_in_cycle = false;

	if (complete) {
		perf_count(_cycles_perf);
	}
}

int NanoRadarNRA15Can::print_status()
{
	const hrt_abstime now = hrt_absolute_time();

	PX4_INFO("interface: can%d (CAN%d), bitrate %s", _bus, _bus + 1,
		 _bitrate_set ? "500 kbit/s" : "not set by the driver");

	if (_have_version) {
		PX4_INFO("firmware: V%u.%u.%u", _version[0], _version[1], _version[2]);

	} else {
		PX4_INFO("firmware: no answer to the version request yet");
	}

	if (_last_cycle != 0) {
		// The driver task may record a cycle after now was taken
		const hrt_abstime age = (now > _last_cycle) ? now - _last_cycle : 0;
		PX4_INFO("last cycle %.0f ms ago: %u targets, nearest %.2f m", (double)(age / 1000),
			 _num_targets, (double)_last_nearest);

		for (uint8_t i = 0; i < _num_targets; i++) {
			PX4_INFO("  target %u: %.2f m, snr %.0f dB, rcs raw %u, vrel raw %u", _targets[i].index,
				 (double)_targets[i].range, (double)_targets[i].snr, _targets[i].rcs_raw, _targets[i].vrel_raw);
		}

	} else if (_last_frame != 0) {
		PX4_INFO("frames received, no cycle yet");

	} else {
		PX4_INFO("no frames received");
	}

	perf_print_counter(_frames_perf);
	perf_print_counter(_cycles_perf);
	perf_print_counter(_missed_perf);
	perf_print_counter(_incomplete_perf);
	perf_print_counter(_timeout_perf);
	perf_print_counter(_read_err_perf);
	return 0;
}

int NanoRadarNRA15Can::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("nra15",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SLOW_DRIVER,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

NanoRadarNRA15Can *NanoRadarNRA15Can::instantiate(int argc, char *argv[])
{
	// Default from the parameter, the command line wins.
	int32_t can_port = 0;
	param_get(param_find("SENS_NRA15_CAN"), &can_port);
	int rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			can_port = atoi(myoptarg);
			break;

		case 'R':
			rotation = atoi(myoptarg);
			break;

		default:
			print_usage("unknown option");
			return nullptr;
		}
	}

	if (can_port < 1 || can_port > 2) {
		PX4_ERR("CAN port %d invalid: set SENS_NRA15_CAN to 1 (CAN1) or 2 (CAN2), or pass -b", (int)can_port);
		return nullptr;
	}

	return new NanoRadarNRA15Can(can_port - 1, static_cast<uint8_t>(rotation));
}

int NanoRadarNRA15Can::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NanoRadarNRA15Can::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the Nanoradar NRA15 24 GHz altimeter radar on CAN (SocketCAN, 500 kbit/s).

Reads the target status (0x70B) and target info (0x70C) of every 25 ms cycle
and publishes the nearest target as distance_sensor. Asks for the firmware
version once at start (0x200 read request).

### Examples
Start on CAN2, facing down:
$ nra15 start -b 2
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nra15", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 1, 2, "CAN port, 1 = CAN1 (can0), 2 = CAN2 (can1); default SENS_NRA15_CAN", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', distance_sensor_s::ROTATION_DOWNWARD_FACING, 0, 25, "Sensor rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int nra15_main(int argc, char *argv[])
{
	return NanoRadarNRA15Can::main(argc, argv);
}
