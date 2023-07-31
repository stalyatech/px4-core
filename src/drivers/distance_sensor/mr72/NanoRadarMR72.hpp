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

/**
 * @file NanoRadarMR72.hpp
 * @author Jessica Stockham <jessica@aerotenna.com>
 * @author Roman Bapst <roman@uaventure.com>
 *
 * Driver for the uLanding radar from Aerotenna
 */

#pragma once

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/filter/filter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/radar_status.h>

using namespace time_literals;

#define MR72_MEASURE_INTERVAL		60_ms
#define MR72_MAX_DISTANCE			40.0f
#define MR72_MIN_DISTANCE			0.2f
#define MR72_AZIMUTH_FOV			112.0f
#define MR72_ELEVATION_FOV			14.0f
#define MR72_RESOLUTION				0.01f

#define MR72_PACKET_STX1			0x54
#define MR72_PACKET_STX2			0x48
#define MR72_PAYLOAD_LEN     		18
#define MR72_PACKET_LEN     		19
#define MR72_BUFFER_LEN     		1024

#define MR72_DIST_INVALID     		0xffff

typedef struct __attribute__((__packed__)) message_frame {
	uint16_t state;
	uint16_t paycnt;
	uint8_t  paybuf[MR72_PAYLOAD_LEN];
	uint8_t  rawbuf[MR72_BUFFER_LEN];
} message_frame_t;

typedef struct __attribute__((__packed__)) target_info {
	uint16_t sector1;
	uint16_t sector2;
	uint16_t sector3;
	uint16_t dist_90;
	uint16_t dist_135;
	uint16_t dist_180;
	uint16_t dist_225;
	uint16_t dist_270;
} target_info_t;

class NanoRadarMR72 : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	NanoRadarMR72(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING);
	~NanoRadarMR72() override;

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:
	enum
	{
		FRAME_STAT_STX1,
		FRAME_STAT_STX2,
		FRAME_STAT_PAYLOAD,
		FRAME_STAT_CRC,
	};

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

	/**
	 * Parse data.
	 */
	int parse(uint8_t inData);

	/**
	 * Get the message from collected data.
	 */
	int message(uint8_t crc);

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	/**
	 * Calculates CRC8.
	 */
	uint8_t crc8(const uint8_t *p, uint8_t len);

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	void Run() override;

	uORB::Publication<radar_status_s> _radar_status_pub{ORB_ID(radar_status)};

	PX4Rangefinder _px4_rangefinder;
	const char *_serial_port{nullptr};
	int _file_descriptor{-1};

	message_frame_t _message{0};
	target_info_t _target_info{0};

	perf_counter_t _cycle_perf{nullptr};
	perf_counter_t _sample_perf{nullptr};
	perf_counter_t _error_perf{nullptr};
	perf_counter_t _packet_perf{nullptr};

	uint16_t _lastDist{0};
	int32_t  _filterType{0};
	filter::Filter * _filter{nullptr};
};
