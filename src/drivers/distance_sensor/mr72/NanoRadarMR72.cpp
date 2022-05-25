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

NanoRadarMR72::NanoRadarMR72(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	/* store port name */
	_serial_port = strdup(port);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_MR72);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);

	_px4_rangefinder.set_min_distance(MR72_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MR72_MAX_DISTANCE);
	_px4_rangefinder.set_fov(math::radians(MR72_FIELD_OF_VIEW));
}

NanoRadarMR72::~NanoRadarMR72()
{
	stop();

	free((char *)_serial_port);
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int NanoRadarMR72::init()
{
	start();

	return PX4_OK;
}

int NanoRadarMR72::message()
{
	return PX4_ERROR;
}//message

int NanoRadarMR72::parse(uint8_t inData)
{
	switch (_message.state)
	{
		case FRAME_STAT_STX1:
		{
			if (inData == MR72_PACKET_STX1) {
				_message.state = FRAME_STAT_STX2;
			}//if
			break;
		}//FRAME_STAT_STX1

		case FRAME_STAT_STX2:
		{
			if (inData == MR72_PACKET_STX2) {
				_message.state  = FRAME_STAT_PAYLOAD;
				_message.paycnt = 0;
			}//if
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
			return message();
		}//FRAME_STAT_CRC
	}//switch

	return PX4_ERROR;
}//parse


int NanoRadarMR72::collect()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int bytes_read = ::read(_file_descriptor, &_message.rawbuf[0], sizeof(_message.rawbuf));
	int updated = 0;
	int msgid;

	for (int index = 0; index < bytes_read; index++) {

		// try to parse frame
		if ((msgid = parse(_message.rawbuf[index])) != PX4_ERROR) {

			// inform the application
			_px4_rangefinder.update(timestamp_sample, _target_info.dist);

			// increment the update counter
			updated++;
		}//if
	}//if

	perf_end(_sample_perf);

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
}

void NanoRadarMR72::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void NanoRadarMR72::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(MR72_MEASURE_INTERVAL, 0);
}

void NanoRadarMR72::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}

void NanoRadarMR72::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
