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

/**
 * @file nra15_can_protocol.hpp
 *
 * Nanoradar NRA15 CAN messages.
 *
 * Plain functions without PX4 dependencies so they can be tested on a host.
 *
 * The CAN model sends the same messages as the UART model (NRA15 user
 * manual, section 5) as 500 kbit/s classic CAN frames with 11 bit
 * identifiers. Every 25 ms cycle carries the sensor status (0x60A) and the
 * target status (0x70B), followed by one target info frame (0x70C) per
 * detected target.
 */

#pragma once

#include <stdint.h>

namespace nra15_can
{

static constexpr uint32_t BITRATE = 500000;

static constexpr uint32_t ID_SENSOR_CONFIG = 0x200;  // to the radar
static constexpr uint32_t ID_SENSOR_BACK   = 0x400;  // reply to 0x200
static constexpr uint32_t ID_SENSOR_STATUS = 0x60A;  // every cycle
static constexpr uint32_t ID_TARGET_STATUS = 0x70B;  // every cycle
static constexpr uint32_t ID_TARGET_INFO   = 0x70C;  // one per target

// 0x200 / 0x400 data types
static constexpr uint8_t DATA_TYPE_VERSION = 2;

struct TargetStatus {
	uint8_t num_targets;
	uint8_t roll_count;  // 0-1-2-3, one step per cycle
};

struct TargetInfo {
	uint8_t index;
	float range;         // m
	uint8_t rcs_raw;     // scale not documented
	uint16_t vrel_raw;   // 11 bits, 0.05 m/s per the manual, sign not documented
	float snr;           // dB
};

struct SensorBack {
	uint8_t data_type;
	bool ok;
	uint8_t param[3];    // for the version: major, minor, patch
};

inline bool parse_target_status(const uint8_t *d, uint8_t len, TargetStatus &out)
{
	if (len < 2) {
		return false;
	}

	out.num_targets = d[0];
	out.roll_count = d[1] & 0x03;
	return true;
}

inline bool parse_target_info(const uint8_t *d, uint8_t len, TargetInfo &out)
{
	if (len < 8) {
		return false;
	}

	out.index = d[0];
	out.rcs_raw = d[1];
	out.range = static_cast<float>((d[2] << 8) | d[3]) * 0.01f;
	out.vrel_raw = static_cast<uint16_t>(((d[5] & 0x07) << 8) | d[6]);
	// The manual gives 1 dB per bit; the UART driver's offset of 127 gives
	// plausible values (0x9D: 30 dB).
	out.snr = static_cast<float>(d[7]) - 127.f;
	return true;
}

inline bool parse_sensor_back(const uint8_t *d, uint8_t len, SensorBack &out)
{
	if (len < 4) {
		return false;
	}

	out.data_type = d[0] & 0x7F;
	out.ok = (d[0] & 0x80) != 0;
	out.param[0] = d[1];
	out.param[1] = d[2];
	out.param[2] = d[3];
	return true;
}

// 0x200 payload asking for the firmware version (read only)
inline void make_version_request(uint8_t d[8])
{
	for (int i = 0; i < 8; i++) {
		d[i] = 0;
	}

	d[0] = DATA_TYPE_VERSION;  // R/W bit 7 = 0: read
}

inline uint8_t next_roll_count(uint8_t roll_count)
{
	return (roll_count + 1) & 0x03;
}

} // namespace nra15_can
