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
 * @file NanoRadarNRA15.hpp
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

using namespace time_literals;

#define NRA15_MEASURE_INTERVAL     	25_ms
#define NRA15_MAX_DISTANCE	        30.0f
#define NRA15_MIN_DISTANCE	        0.1f
#define NRA15_FIELD_OF_VIEW        	37.0f

#define NRA15_PACKET_STX1			0xAA
#define NRA15_PACKET_STX2			0xAA
#define NRA15_PACKET_ETX1			0x55
#define NRA15_PACKET_ETX2			0x55
#define NRA15_PAYLOAD_LEN     		8
#define NRA15_PACKET_LEN     		14
#define NRA15_BUFFER_LEN     		256

typedef struct __attribute__((__packed__)) message_frame {
	uint16_t state;
	uint16_t paycnt;
	uint16_t msgid;
	uint8_t  paybuf[NRA15_PAYLOAD_LEN];
	uint8_t  rawbuf[NRA15_BUFFER_LEN];
} message_frame_t;

typedef struct __attribute__((__packed__)) sensor_conf {
	uint8_t  type;
	uint8_t  rw;
	uint32_t param;
} sensor_conf_t;

typedef struct __attribute__((__packed__)) sensor_back {
	uint8_t  type;
	uint8_t  result;
	uint32_t param;
} sensor_back_t;

typedef struct __attribute__((__packed__)) sensor_stat {
	uint8_t actl_mode;
	uint8_t rollcnt;
	uint8_t cfgstat;
} sensor_stat_t;

typedef struct __attribute__((__packed__)) target_stat {
	uint8_t number;
	uint8_t rollcnt;
} target_stat_t;

typedef struct __attribute__((__packed__)) target_info {
	uint8_t id;
	uint8_t rollcnt;
	uint8_t snr;
	float 	vrel;
	float 	rcs;
	float 	dist;
} target_info_t;

class NanoRadarNRA15 : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	NanoRadarNRA15(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~NanoRadarNRA15() override;

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
		FRAME_STAT_MSG1,
		FRAME_STAT_MSG2,
		FRAME_STAT_PAYLOAD,
		FRAME_STAT_ETX1,
		FRAME_STAT_ETX2,
	};

	enum
	{
		MSGID_SENSOR_CONFIG = 0x200,
		MSGID_SENSOR_BACK 	= 0x400,
		MSGID_SENSOR_STATUS = 0x60A,
		MSGID_TARGET_STATUS = 0x70B,
		MSGID_TARGET_INFO 	= 0x70C,
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
	int message();

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	void Run() override;

	PX4Rangefinder _px4_rangefinder;
	const char *_serial_port{nullptr};
	int _file_descriptor{-1};

	message_frame_t _message{0};
	sensor_conf_t _sensor_conf{0};
	sensor_back_t _sensor_back{0};
	sensor_stat_t _sensor_stat{0};
	target_stat_t _target_stat{0};
	target_info_t _target_info{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};

};
