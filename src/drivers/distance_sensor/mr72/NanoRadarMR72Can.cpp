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

#include "NanoRadarMR72Can.hpp"
#include "../nanoradar_socketcan.hpp"

#include <poll.h>

#include <arch/board/board.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

using namespace mr72_can;

NanoRadarMR72Can::NanoRadarMR72Can(int bus, uint8_t sensor_id, uint8_t rotation, uint32_t bitrate, bool raw,
				   bool probe) :
	_bus(bus),
	_sensor_id(sensor_id),
	_bitrate(bitrate),
	_raw(raw),
	_probe(probe),
	_px4_rangefinder(0, rotation)
{
	device::Device::DeviceId device_id{};
	// PX4 has no plain CAN bus type; the UAVCAN one stands for "on a CAN bus".
	device_id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;
	device_id.devid_s.bus = bus;
	device_id.devid_s.address = sensor_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_MR72;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_MR72);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_min_distance(MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MAX_DISTANCE);
	_px4_rangefinder.set_hfov(math::radians(AZIMUTH_FOV));
	_px4_rangefinder.set_vfov(math::radians(ELEVATION_FOV));

	_device_id = device_id.devid;
	_objects.device_id = _device_id;

	param_get(param_find("SENS_MR72_MASK"), &_sector_mask);
}

NanoRadarMR72Can::~NanoRadarMR72Can()
{
	close_socket();
	perf_free(_frames_perf);
	perf_free(_cycles_perf);
	perf_free(_incomplete_perf);
	perf_free(_timeout_perf);
	perf_free(_read_err_perf);
}

int NanoRadarMR72Can::open_socket()
{
	// Only the messages of this radar, unless every frame is wanted
	const struct can_filter filters[] = {
		nanoradar_can::standard_id_filter(message_id(ID_OBJECT_STATUS, _sensor_id)),
		nanoradar_can::standard_id_filter(message_id(ID_OBJECT_GENERAL, _sensor_id)),
		nanoradar_can::standard_id_filter(message_id(ID_RADAR_STATE, _sensor_id)),
		nanoradar_can::standard_id_filter(message_id(ID_VERSION, _sensor_id)),
	};

	_sock = nanoradar_can::open_socket(_bus, _bitrate, _raw ? nullptr : filters,
					   sizeof(filters) / sizeof(filters[0]), _bitrate_set);

	if (_sock < 0) {
		return PX4_ERROR;
	}

	if (_probe) {
		// Any powered node on the bus acknowledges a valid frame, whatever the
		// identifier: a missing ACK shows up in the controller error state.
		struct can_frame frame {};
		frame.can_id = 0x7f0;
		frame.can_dlc = 0;

		if (write(_sock, &frame, sizeof(frame)) != sizeof(frame)) {
			PX4_WARN("probe frame not sent (%d)", errno);
		}
	}

	return PX4_OK;
}

void NanoRadarMR72Can::close_socket()
{
	if (_sock >= 0) {
		close(_sock);
		_sock = -1;
	}
}

void NanoRadarMR72Can::run()
{
	if (open_socket() != PX4_OK) {
		return;
	}

	struct pollfd fds {};
	fds.fd = _sock;
	fds.events = POLLIN;

	bool timed_out = false;

	while (!should_exit()) {
		const int ret = poll(&fds, 1, 100);
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

	close_socket();
}

void NanoRadarMR72Can::handle_frame(const struct can_frame &frame, hrt_abstime now)
{
	_last_frame = now;
	perf_count(_frames_perf);

	if (_raw) {
		record_raw_frame(frame);
	}

	if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) {
		return;
	}

	const uint32_t id = frame.can_id & CAN_SFF_MASK;
	const uint8_t len = frame.can_dlc;

	if (id == message_id(ID_OBJECT_STATUS, _sensor_id)) {
		ObjectStatus status{};

		if (parse_object_status(frame.data, len, status)) {
			start_cycle(status, now);
		}

	} else if (id == message_id(ID_OBJECT_GENERAL, _sensor_id)) {
		Object obj{};

		if (_in_cycle && parse_object(frame.data, len, obj)) {
			add_object(obj);

			if (_received >= _expected) {
				publish_cycle(true);
			}
		}

	} else if (id == message_id(ID_RADAR_STATE, _sensor_id)) {
		_have_state = parse_radar_state(frame.data, len, _state);

	} else if (id == message_id(ID_VERSION, _sensor_id)) {
		_have_version = parse_version(frame.data, len, _version);
	}
}

void NanoRadarMR72Can::start_cycle(const ObjectStatus &status, hrt_abstime now)
{
	if (_in_cycle) {
		// A new header before all objects of the previous cycle arrived.
		perf_count(_incomplete_perf);
		publish_cycle(false);
	}

	_objects = radar_objects_s{};
	_objects.device_id = _device_id;
	_in_cycle = true;
	_cycle_start = now;
	_meas_count = status.meas_count;
	_expected = status.num_objects;
	_received = 0;

	if (_expected == 0) {
		publish_cycle(true);
	}
}

void NanoRadarMR72Can::add_object(const Object &obj)
{
	_received++;

	if (!object_valid(obj, MIN_DISTANCE) || _objects.count >= radar_objects_s::MAX_OBJECTS) {
		return;
	}

	const uint8_t i = _objects.count++;
	_objects.id[i] = obj.id;
	_objects.dist_long[i] = obj.dist_long;
	_objects.dist_lat[i] = obj.dist_lat;
	_objects.vrel_long[i] = obj.vrel_long;
	_objects.vrel_lat[i] = obj.vrel_lat;
	_objects.rcs[i] = obj.rcs;
	_objects.sector[i] = obj.sector;
}

void NanoRadarMR72Can::publish_cycle(bool complete)
{
	float sector_nearest[3] {-1.0f, -1.0f, -1.0f};
	float nearest = -1.0f;

	for (uint8_t i = 0; i < _objects.count; i++) {
		const float d = _objects.dist_long[i];
		const uint8_t s = _objects.sector[i];

		if (s >= 1 && s <= 3 && (sector_nearest[s - 1] < 0.0f || d < sector_nearest[s - 1])) {
			sector_nearest[s - 1] = d;
		}

		const bool in_mask = (s == 0) || (_sector_mask & (1 << (s - 1)));

		if (in_mask && (nearest < 0.0f || d < nearest)) {
			nearest = d;
		}
	}

	_objects.timestamp_sample = _cycle_start;
	_objects.meas_count = _meas_count;
	_objects.reported = _expected;
	_objects.timestamp = hrt_absolute_time();
	_radar_objects_pub.publish(_objects);

	radar_status_s status{};
	status.timestamp = _objects.timestamp;

	for (int i = 0; i < 3; i++) {
		status.sector[i] = sector_nearest[i];
	}

	// The CAN object list has no fixed-angle sector distances.
	for (int i = 0; i < 5; i++) {
		status.distance[i] = -1.0f;
	}

	_radar_status_pub.publish(status);

	if (nearest > 0.0f) {
		_px4_rangefinder.update(_cycle_start, nearest, 100);

	} else {
		// Nothing in range: report the far limit with zero quality.
		_px4_rangefinder.update(_cycle_start, MAX_DISTANCE, 0);
	}

	_last_nearest = nearest;
	_last_cycle = _cycle_start;
	_in_cycle = false;

	if (complete) {
		perf_count(_cycles_perf);
	}
}

void NanoRadarMR72Can::record_raw_frame(const struct can_frame &frame)
{
	int i = 0;

	while (i < _raw_id_count && _raw_ids[i] != frame.can_id) {
		i++;
	}

	if (i == _raw_id_count) {
		if (_raw_id_count == MAX_RAW_IDS) {
			_raw_overflow++;
			return;
		}

		_raw_ids[i] = frame.can_id;
		_raw_counts[i] = 0;
		_raw_id_count++;
	}

	_raw_counts[i]++;
	_raw_dlc[i] = math::min(frame.can_dlc, static_cast<uint8_t>(8));
	memcpy(_raw_last[i], frame.data, _raw_dlc[i]);
}

void NanoRadarMR72Can::print_controller_status() const
{
#if defined(CONFIG_ARCH_CHIP_STM32H7)
	// FDCAN registers (RM0433): CCCR 0x18, NBTP 0x1C, ECR 0x40, PSR 0x44, IR 0x50, RXF0S 0xA4, RXF1S 0xB4
	const uintptr_t base = (_bus == 0) ? 0x4000a000 : 0x4000a400;
	const uint32_t cccr = *reinterpret_cast<volatile uint32_t *>(base + 0x18);
	const uint32_t nbtp = *reinterpret_cast<volatile uint32_t *>(base + 0x1c);
	const uint32_t ecr = *reinterpret_cast<volatile uint32_t *>(base + 0x40);
	const uint32_t psr = *reinterpret_cast<volatile uint32_t *>(base + 0x44);
	const uint32_t ir = *reinterpret_cast<volatile uint32_t *>(base + 0x50);
	const uint32_t rxf0s = *reinterpret_cast<volatile uint32_t *>(base + 0xa4);
	const uint32_t rxf1s = *reinterpret_cast<volatile uint32_t *>(base + 0xb4);

	const uint32_t brp = ((nbtp >> 16) & 0x1ff) + 1;
	const uint32_t tseg1 = ((nbtp >> 8) & 0xff) + 1;
	const uint32_t tseg2 = (nbtp & 0x7f) + 1;
	const uint32_t bitrate = STM32_HSE_FREQUENCY / (brp * (1 + tseg1 + tseg2));

	static const char *const lec_names[8] {"none", "stuff", "form", "ack", "bit1", "bit0", "crc", "no change"};
	static const char *const act_names[4] {"sync", "idle", "receiving", "transmitting"};

	PX4_INFO("controller: %u bit/s (brp %u, seg %u+%u), %s%s%s",
		 static_cast<unsigned>(bitrate), static_cast<unsigned>(brp), static_cast<unsigned>(tseg1),
		 static_cast<unsigned>(tseg2), (cccr & 1) ? "INIT " : "", (cccr & (1 << 5)) ? "MONITOR " : "",
		 (cccr & (1 << 7)) ? "TEST" : "");
	PX4_INFO("errors: tx %u, rx %u, last %s, state %s%s%s%s",
		 static_cast<unsigned>(ecr & 0xff), static_cast<unsigned>((ecr >> 8) & 0x7f),
		 lec_names[psr & 7], act_names[(psr >> 3) & 3],
		 (psr & (1 << 5)) ? ", error passive" : "", (psr & (1 << 6)) ? ", warning" : "",
		 (psr & (1 << 7)) ? ", BUS OFF" : "");
	PX4_INFO("rx fifo: 0 fill %u%s, 1 fill %u%s, irq 0x%08x",
		 static_cast<unsigned>(rxf0s & 0x7f), (rxf0s & (1 << 25)) ? " LOST" : "",
		 static_cast<unsigned>(rxf1s & 0x7f), (rxf1s & (1 << 25)) ? " LOST" : "",
		 static_cast<unsigned>(ir));
#endif
}

int NanoRadarMR72Can::print_status()
{
	const hrt_abstime now = hrt_absolute_time();

	PX4_INFO("interface: can%d (CAN%d), sensor id %u, bitrate %u %s%s", _bus, _bus + 1, _sensor_id,
		 static_cast<unsigned>(_bitrate), _bitrate_set ? "set" : "not set by the driver",
		 _raw ? ", raw mode" : "");
	print_controller_status();

	if (_raw) {
		for (int i = 0; i < _raw_id_count; i++) {
			char hex[3 * 8 + 1] {};

			for (int b = 0; b < _raw_dlc[i]; b++) {
				snprintf(&hex[3 * b], 4, "%02x ", _raw_last[i][b]);
			}

			PX4_INFO("  id 0x%03x%s: %u frames, last [%u] %s", static_cast<unsigned>(_raw_ids[i] & CAN_EFF_MASK),
				 (_raw_ids[i] & CAN_EFF_FLAG) ? " (ext)" : "", static_cast<unsigned>(_raw_counts[i]),
				 static_cast<unsigned>(_raw_dlc[i]), hex);
		}

		if (_raw_overflow > 0) {
			PX4_INFO("  %u frames with further identifiers", static_cast<unsigned>(_raw_overflow));
		}
	}

	if (_have_version) {
		PX4_INFO("firmware: V%u.%u.%u", _version.major, _version.minor, _version.patch);
	}

	if (_have_state) {
		PX4_INFO("radar state: id %u, max %u m, output %u, sort %u, power %u, rcs threshold %u, nvm read %s write %s",
			 _state.sensor_id, _state.max_distance_m, _state.output_type, _state.sort_index,
			 _state.radar_power, _state.rcs_threshold,
			 _state.nvm_read_ok ? "ok" : "failed", _state.nvm_write_ok ? "ok" : "failed");
	}

	if (_last_cycle != 0) {
		// The driver task may record a cycle after now was taken
		const hrt_abstime age = (now > _last_cycle) ? now - _last_cycle : 0;
		PX4_INFO("last cycle %.0f ms ago: #%u, %u reported, %u valid, nearest %.2f m",
			 (double)(age / 1000), _objects.meas_count, _objects.reported, _objects.count,
			 (double)_last_nearest);

		for (uint8_t i = 0; i < _objects.count; i++) {
			PX4_INFO("  obj %3u: sector %u  long %7.2f m  lat %6.2f m  v %6.2f m/s  rcs %5.1f",
				 _objects.id[i], _objects.sector[i], (double)_objects.dist_long[i], (double)_objects.dist_lat[i],
				 (double)_objects.vrel_long[i], (double)_objects.rcs[i]);
		}

	} else if (_last_frame != 0) {
		PX4_INFO("frames received, no object cycle yet");

	} else {
		PX4_INFO("no frames received");
	}

	perf_print_counter(_frames_perf);
	perf_print_counter(_cycles_perf);
	perf_print_counter(_incomplete_perf);
	perf_print_counter(_timeout_perf);
	perf_print_counter(_read_err_perf);
	return 0;
}

int NanoRadarMR72Can::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mr72",
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

NanoRadarMR72Can *NanoRadarMR72Can::instantiate(int argc, char *argv[])
{
	// Defaults from the parameters, the command line wins.
	int32_t can_port = 0;
	int32_t sensor_id = 0;
	param_get(param_find("SENS_MR72_CAN"), &can_port);
	param_get(param_find("SENS_MR72_ID"), &sensor_id);
	int rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
	int32_t bitrate = BITRATE;
	bool raw = false;
	bool probe = false;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:i:R:B:rp", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			can_port = atoi(myoptarg);
			break;

		case 'i':
			sensor_id = atoi(myoptarg);
			break;

		case 'R':
			rotation = atoi(myoptarg);
			break;

		case 'B':
			bitrate = atoi(myoptarg);
			break;

		case 'r':
			raw = true;
			break;

		case 'p':
			probe = true;
			break;

		default:
			print_usage("unknown option");
			return nullptr;
		}
	}

	if (can_port < 1 || can_port > 2) {
		PX4_ERR("CAN port %d invalid: set SENS_MR72_CAN to 1 (CAN1) or 2 (CAN2), or pass -b", (int)can_port);
		return nullptr;
	}

	if (sensor_id < 0 || sensor_id > MAX_SENSOR_ID) {
		PX4_ERR("sensor id %d invalid (0...%u)", (int)sensor_id, MAX_SENSOR_ID);
		return nullptr;
	}

	if (bitrate < 10000 || bitrate > 1000000) {
		PX4_ERR("bitrate %d invalid (10000...1000000)", (int)bitrate);
		return nullptr;
	}

	return new NanoRadarMR72Can(can_port - 1, static_cast<uint8_t>(sensor_id), static_cast<uint8_t>(rotation),
				    static_cast<uint32_t>(bitrate), raw, probe);
}

int NanoRadarMR72Can::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NanoRadarMR72Can::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the Nanoradar MR72 77 GHz radar on CAN (SocketCAN, 500 kbit/s).

Reads the object list (0x60A header + 0x60B per object), the radar state
(0x201) and the firmware version (0x700). Publishes the full list as
radar_objects, the nearest object within SENS_MR72_MASK as distance_sensor,
and the nearest object per sector as radar_status.

### Examples
Start on CAN1 for a radar with SensorID 0:
$ mr72 start -b 1 -i 0
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mr72", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 1, 2, "CAN port, 1 = CAN1 (can0), 2 = CAN2 (can1); default SENS_MR72_CAN", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 7, "Radar SensorID; default SENS_MR72_ID", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', distance_sensor_s::ROTATION_FORWARD_FACING, 0, 25, "Sensor rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('B', 500000, 10000, 1000000, "CAN bitrate in bit/s", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Raw mode: accept every frame and list the identifiers seen", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('p', "Send one empty frame (id 0x7f0) to check that a node acknowledges", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mr72_main(int argc, char *argv[])
{
	return NanoRadarMR72Can::main(argc, argv);
}
