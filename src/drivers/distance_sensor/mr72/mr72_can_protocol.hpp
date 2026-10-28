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
 * @file mr72_can_protocol.hpp
 *
 * Nanoradar MR72 CAN messages (MR72 Radar Communication Protocol V1.2).
 *
 * Plain functions without PX4 dependencies so they can be tested on a host.
 *
 * The radar sends 500 kbit/s classic CAN with 11 bit identifiers. Every
 * message identifier is offset by SensorID * 0x10 (SensorID 0...7).
 * Signals are packed big endian (Motorola); the start bits in the protocol
 * document number the least significant bit. The byte formulas below follow
 * the document's own decoding examples. The layout matches the Continental
 * ARS408 object list, which the MR72 copies; for the 0x60A cycle counter the
 * document's start bit is wrong and the ARS408 position (bytes 1 and 2) is
 * used.
 */

#pragma once

#include <stdint.h>

namespace mr72_can
{

static constexpr uint32_t BITRATE = 500000;

// Message identifiers for SensorID 0
static constexpr uint32_t ID_RADAR_CFG      = 0x200;  // to the radar
static constexpr uint32_t ID_RADAR_STATE    = 0x201;  // 1 Hz
static constexpr uint32_t ID_OBJECT_STATUS  = 0x60A;  // cycle header
static constexpr uint32_t ID_OBJECT_GENERAL = 0x60B;  // one per object
static constexpr uint32_t ID_VERSION        = 0x700;  // 1 Hz heartbeat

static constexpr uint8_t MAX_SENSOR_ID = 7;

/** Identifier of message @p base for radar @p sensor_id. */
static constexpr uint32_t message_id(uint32_t base, uint8_t sensor_id)
{
	return base + static_cast<uint32_t>(sensor_id) * 0x10u;
}

/** 0x60A: header sent at the start of every measurement cycle. */
struct ObjectStatus {
	uint8_t num_objects;        // objects that follow as 0x60B
	uint16_t meas_count;        // cycle counter
	uint8_t interface_version;
};

/** 0x60B: one tracked object. */
struct Object {
	uint8_t id;                 // track id, 0...255, stable while tracked
	float dist_long;            // m, along the boresight (-500 ... 1138.2)
	float dist_lat;             // m, lateral (-204.6 ... 204.8)
	float vrel_long;            // m/s, radial (-128 ... 127.75)
	float vrel_lat;             // m/s, lateral (-64 ... 63.75)
	float rcs;                  // dBm2 (-64 ... 63.5)
	uint8_t dyn_prop;           // dynamic property, 0 on current firmware
	uint8_t sector;             // 1...3, sector of the object
};

/** 0x201: configuration the radar is running with, sent once a second. */
struct RadarState {
	bool nvm_read_ok;
	bool nvm_write_ok;
	uint16_t max_distance_m;
	uint8_t sensor_id;
	uint8_t sort_index;         // 0 none, 1 range, 2 RCS
	uint8_t radar_power;        // 0 standard, 1...3: -3/-6/-9 dB
	uint8_t output_type;        // 0 none, 1 objects, 2 clusters
	uint8_t rcs_threshold;      // 0 standard, 1 high sensitivity
};

/** 0x700: firmware version. */
struct Version {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
};

inline bool parse_object_status(const uint8_t *d, uint8_t len, ObjectStatus &out)
{
	if (len < 4) {
		return false;
	}

	out.num_objects = d[0];
	out.meas_count = static_cast<uint16_t>((d[1] << 8) | d[2]);
	out.interface_version = d[3] >> 4;
	return true;
}

inline bool parse_object(const uint8_t *d, uint8_t len, Object &out)
{
	if (len < 8) {
		return false;
	}

	const uint32_t dist_long = (static_cast<uint32_t>(d[1]) << 5) | (d[2] >> 3);          // 13 bit
	const uint32_t dist_lat  = (static_cast<uint32_t>(d[2] & 0x07) << 8) | d[3];          // 11 bit
	const uint32_t vrel_long = (static_cast<uint32_t>(d[4]) << 2) | (d[5] >> 6);          // 10 bit
	const uint32_t vrel_lat  = (static_cast<uint32_t>(d[5] & 0x3F) << 3) | (d[6] >> 5);   // 9 bit

	out.id        = d[0];
	out.dist_long = dist_long * 0.2f - 500.0f;
	out.dist_lat  = dist_lat * 0.2f - 204.6f;
	out.vrel_long = vrel_long * 0.25f - 128.0f;
	out.vrel_lat  = vrel_lat * 0.25f - 64.0f;
	out.dyn_prop  = d[6] & 0x07;
	out.sector    = (d[6] >> 3) & 0x03;
	out.rcs       = d[7] * 0.5f - 64.0f;
	return true;
}

inline bool parse_radar_state(const uint8_t *d, uint8_t len, RadarState &out)
{
	if (len < 8) {
		return false;
	}

	out.nvm_read_ok    = (d[0] >> 6) & 0x01;
	out.nvm_write_ok   = (d[0] >> 7) & 0x01;
	out.max_distance_m = static_cast<uint16_t>(((d[1] << 2) | (d[2] >> 6)) * 2);
	out.sensor_id      = d[4] & 0x07;
	out.sort_index     = (d[4] >> 4) & 0x07;
	out.radar_power    = static_cast<uint8_t>(((d[3] & 0x03) << 1) | (d[4] >> 7));
	out.output_type    = (d[5] >> 2) & 0x03;
	out.rcs_threshold  = (d[7] >> 2) & 0x07;
	return true;
}

inline bool parse_version(const uint8_t *d, uint8_t len, Version &out)
{
	if (len < 3) {
		return false;
	}

	out.major = d[0];
	out.minor = d[1];
	out.patch = d[2];
	return true;
}

/**
 * True when @p obj describes a real target. The three-sector firmware
 * sends a slot for every sector and fills it with zeros when the sector is
 * empty, which decodes to a negative or near zero distance.
 */
inline bool object_valid(const Object &obj, float min_distance)
{
	return obj.dist_long >= min_distance;
}

} // namespace mr72_can
