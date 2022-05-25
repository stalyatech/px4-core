/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include "Bosch_BMI088_Gyroscope_Registers.hpp"

using namespace device;

namespace Bosch::BMI088::Gyroscope
{

class BMI088_Gyroscope
{
public:
	BMI088_Gyroscope(Device *interface, enum Rotation rotation);
	~BMI088_Gyroscope() {};

	int  Probe();
	int  Reset();
	int  Check();
	int  ReadData(int16_t *gyro);
	int  ReadFIFO(const hrt_abstime &timestamp_sample, sensor_gyro_fifo_s &gyro);
	bool Configure();

	inline void Update(const hrt_abstime &timestamp_sample, float x, float y, float z) {
		_px4_gyro.update(timestamp_sample, x, y, z);
	}

	inline void UpdateFIFO(sensor_gyro_fifo_s &gyro) {
		_px4_gyro.updateFIFO(gyro);
	}

	inline void SetErrorCount(uint32_t error_count) {
		_px4_gyro.set_error_count(error_count);
	}

	inline void SetTemperature(float temperature) {
		_px4_gyro.set_temperature(temperature);
	}

private:
	Device 	       *_interface{nullptr};
	PX4Gyroscope 	_px4_gyro;

	// Sensor Configuration
	static constexpr uint32_t RATE{2000};
	static constexpr float FIFO_SAMPLE_DT{1e6f / RATE};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{5};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                         | Set bits, Clear bits
		{ Register::GYRO_RANGE,             GYRO_RANGE_BIT::gyro_range_2000_dps, 0 },
		{ Register::GYRO_BANDWIDTH,         GYRO_BANDWIDTH_BIT::gyro_bw_230_odr_2000_Hz, 0 },
		{ Register::GYRO_INT_CTRL,          GYRO_INT_CTRL_BIT::data_en, 0 },
		{ Register::INT3_INT4_IO_CONF,      INT3_INT4_IO_CONF_BIT::Int3_lvl, INT3_INT4_IO_CONF_BIT::Int3_od },
		{ Register::INT3_INT4_IO_MAP,       INT3_INT4_IO_MAP_BIT::Int3_data, 0 },
	};

	void 	ConfigureGyro();
	uint8_t RegisterRead(Register reg);
	int 	RegisterWrite(Register reg, const uint8_t value);
	void 	RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	bool 	RegisterCheck(const register_config_t &reg_cfg);
	int 	StreamRead(Register reg, void *data, unsigned count);
	int 	StreamWrite(Register reg, const void *data, unsigned count);
};

} // namespace Bosch::BMI088::Gyroscope
