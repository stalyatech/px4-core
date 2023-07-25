/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "NanoRadarMR72.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

using namespace filter;

static const uint8_t crc8_table[] =
{
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
	0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
	0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
	0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
	0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
	0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
	0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
	0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
	0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
	0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
	0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
	0xfa, 0xfd, 0xf4, 0xf3
};

NanoRadarMR72::NanoRadarMR72(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_sample_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": sample")),
	_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": error")),
	_packet_perf(perf_alloc(PC_COUNT, MODULE_NAME": packet"))
{
	/* store port name */
	_serial_port = strdup(port);

	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.devtype  = DRV_DIST_DEVTYPE_MR72;

	uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'
	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_MR72);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);

	_px4_rangefinder.set_min_distance(MR72_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MR72_MAX_DISTANCE);
	_px4_rangefinder.set_hfov(math::radians(MR72_AZIMUTH_FOV));
	_px4_rangefinder.set_vfov(math::radians(MR72_ELEVATION_FOV));
}

NanoRadarMR72::~NanoRadarMR72()
{
	stop();

	delete _filter;
	free((char *)_serial_port);
	perf_free(_cycle_perf);
	perf_free(_sample_perf);
	perf_free(_error_perf);
	perf_free(_packet_perf);
}

int NanoRadarMR72::init()
{
	param_get(param_find("SENS_MR72_FILT"), &_filterType);

	if (_filter != nullptr) {
		delete _filter;
		_filter = nullptr;
	}
	_filter = new Filter(_filterType);

	start();

	return PX4_OK;
}//init

uint8_t NanoRadarMR72::crc8(const uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

	while (len--) {
		i = (crc ^ *p++) & 0xFF;
		crc = (crc8_table[i] ^ (crc << 8)) & 0xFF;
	}

	return crc & 0xFF;
}//crc8

int NanoRadarMR72::message(uint8_t crc)
{
	// calculate and check the crc
	if (crc8(_message.paybuf, MR72_PAYLOAD_LEN) != crc) {

		// performance counter for checksum error
		perf_count(_error_perf);

		return PX4_ERROR;
	}

	// parse all data
	_target_info.sector1 =  (_message.paybuf[16] << 8) | _message.paybuf[17];
	_target_info.sector2 =  (_message.paybuf[ 2] << 8) | _message.paybuf[ 3];
	_target_info.sector3 =  (_message.paybuf[ 4] << 8) | _message.paybuf[ 5];

	_target_info.dist_90  =  (_message.paybuf[ 6] << 8) | _message.paybuf[ 7];
	_target_info.dist_135 =  (_message.paybuf[ 8] << 8) | _message.paybuf[ 9];
	_target_info.dist_180 =  (_message.paybuf[10] << 8) | _message.paybuf[11];
	_target_info.dist_225 =  (_message.paybuf[12] << 8) | _message.paybuf[13];
	_target_info.dist_270 =  (_message.paybuf[14] << 8) | _message.paybuf[15];

	return PX4_OK;
}//message

int NanoRadarMR72::parse(uint8_t inData)
{
	switch (_message.state)
	{
		case FRAME_STAT_STX1:
		{
			_message.paycnt = 0;
			if (inData == MR72_PACKET_STX1) {
				_message.paybuf[_message.paycnt++] = inData;
				_message.state = FRAME_STAT_STX2;
			}//if
			break;
		}//FRAME_STAT_STX1

		case FRAME_STAT_STX2:
		{
			if (inData == MR72_PACKET_STX2) {
				_message.paybuf[_message.paycnt++] = inData;
				_message.state  = FRAME_STAT_PAYLOAD;
			} else {
				_message.state = FRAME_STAT_STX1;
			}
			break;
		}//FRAME_STAT_STX2

		case FRAME_STAT_PAYLOAD:
		{
			_message.paybuf[_message.paycnt++] = inData;
			if (_message.paycnt == MR72_PAYLOAD_LEN) {
				_message.state = FRAME_STAT_CRC;
			}
			break;
		}//FRAME_STAT_PAYLOAD

		case FRAME_STAT_CRC:
		{
			_message.state = FRAME_STAT_STX1;
			return message(inData);
		}//FRAME_STAT_CRC
	}//switch

	return PX4_ERROR;
}//parse

int NanoRadarMR72::collect()
{
	// start the performance counter
	perf_begin(_cycle_perf);

	// get the current time stamp
	const hrt_abstime now = hrt_absolute_time();

	// read the data from device
	int bytes_read = ::read(_file_descriptor, &_message.rawbuf[0], sizeof(_message.rawbuf));
	int updated = 0;

	// try to parse the data
	for (int index = 0; index < bytes_read; index++) {

		// performance counter for any packet
		perf_count(_packet_perf);

		// try to parse frame
		if (parse(_message.rawbuf[index]) == PX4_OK) {

			// performance counter for data sample
			perf_count(_sample_perf);

			// nearest object distance
			uint32_t dist = 0xffffffff;

			// check the sector 1 for nearest object
			if (_target_info.sector1 != MR72_DIST_INVALID) {
				if (_target_info.sector1 < dist) {
					dist = _target_info.sector1;
				}
			}

			// check the sector 2 for nearest object
			if (_target_info.sector2 != MR72_DIST_INVALID) {
				if (_target_info.sector2 < dist) {
					dist = _target_info.sector2;
				}
			}

			// check the sector 3 for nearest object
			if (_target_info.sector3 != MR72_DIST_INVALID) {
				if (_target_info.sector3 < dist) {
					dist = _target_info.sector3;
				}
			}

			// inform the application
			if (dist < 0xffff) {
				// increment the update counter
				updated++;

				// update the range finder data
				_px4_rangefinder.update(now, _filter->insert(0, dist * MR72_RESOLUTION));
			}

			// prepare the nanoradar status
			struct radar_status_s radar_status;
			radar_status.timestamp 	 = now;
			radar_status.sector  [0] = (_target_info.sector1  != 0xffff) ? (_target_info.sector1  * MR72_RESOLUTION) : (-1.0f);
			radar_status.sector  [1] = (_target_info.sector2  != 0xffff) ? (_target_info.sector2  * MR72_RESOLUTION) : (-1.0f);
			radar_status.sector  [2] = (_target_info.sector3  != 0xffff) ? (_target_info.sector3  * MR72_RESOLUTION) : (-1.0f);
			radar_status.distance[0] = (_target_info.dist_90  != 0xffff) ? (_target_info.dist_90  * MR72_RESOLUTION) : (-1.0f);
			radar_status.distance[1] = (_target_info.dist_135 != 0xffff) ? (_target_info.dist_135 * MR72_RESOLUTION) : (-1.0f);
			radar_status.distance[2] = (_target_info.dist_180 != 0xffff) ? (_target_info.dist_180 * MR72_RESOLUTION) : (-1.0f);
			radar_status.distance[3] = (_target_info.dist_225 != 0xffff) ? (_target_info.dist_225 * MR72_RESOLUTION) : (-1.0f);
			radar_status.distance[4] = (_target_info.dist_180 != 0xffff) ? (_target_info.dist_180 * MR72_RESOLUTION) : (-1.0f);
			_radar_status_pub.publish(radar_status);
		}//if
	}//if

	// end the performance counter
	perf_end(_cycle_perf);

	return (updated) ? PX4_OK : -EAGAIN;
}

int NanoRadarMR72::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_serial_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	termios uart_config{};

	// Store the current port configuration. attributes.
	if (tcgetattr(_file_descriptor, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_file_descriptor);
		_file_descriptor = -1;
		return PX4_ERROR;
	}

	// Clear: data bit size, two stop bits, parity, hardware flow control.
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("opened UART port %s", _serial_port);

	return PX4_OK;
}//open_serial_port

void NanoRadarMR72::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}//Run

void NanoRadarMR72::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(MR72_MEASURE_INTERVAL, 0);
}//start

void NanoRadarMR72::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}//stop

void NanoRadarMR72::print_info()
{
	PX4_INFO("SEC1:%d, SEC2:%d, SEC3:%d, D90:%d, D135:%d, D180:%d, D225:%d, D270:%d", _target_info.sector1,
																					  _target_info.sector2,
																					  _target_info.sector3,
																					  _target_info.dist_90,
																					  _target_info.dist_135,
																					  _target_info.dist_180,
																					  _target_info.dist_225,
																					  _target_info.dist_270);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_packet_perf);
	perf_print_counter(_error_perf);
}//print_info
